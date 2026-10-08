# HIM RL 真机接入与 Jetson Orin NX 移植

本包将 Terrain `model_16099.pt` 导出的完整 HIM 策略接入 ROS 2 Humble / ros2_control。
软件路径为：关节反馈与 IMU → 45 维观测 → 6 帧历史 → ONNX → 12 个关节位置目标 → 电机侧 PD。
`custom_dog_control` 的 NMPC/WBC 仍可独立使用；本包不把 RL 输出解释为接触力或 WBC 前馈力矩。

模型在独立的 [himloco_custom_dog 部署目录](https://github.com/hsc13576717115/himloco_custom_dog/tree/feat/terrain-16099-deployment/deployment/terrain_16099)，
包含完整/分离 ONNX、TorchScript、原模型参考输出、哈希与验证记录。这里实现的是软件接入，
并不表示 NX 板上、真实串口时序或机器人行走已经验收。

## 模块与接口

| 模块 | 职责 |
| --- | --- |
| `PolicyCore` | 训练一致的关节重排、观测缩放、重力投影、历史缓存与动作解码；不依赖 ROS |
| `OnnxPolicy` | CPU FP32 推理、预分配张量、输入/输出名称、类型、形状与有限性校验 |
| `RlController` | ros2_control 接口、50 Hz 后台推理、模式切换、健康检查与混合 PD 输出 |
| `custom_dog_control/UnitreeSystemInterface` | 复用四路串口驱动、标定、方向与减速比转换 |
| `policy_probe` | C++ 实际部署后端回放与时延测量；不访问机器人 |
| `scripts/install_onnxruntime.sh` | 按 x86_64 / aarch64 下载固定版本 SDK，并校验官方 SHA256 |
| `scripts/build_jetson.sh` | 构建模型、串口、IMU、硬件驱动与 RL，独立输出到 `install_rl` |

推理只在一个后台线程执行。硬件控制周期与模型周期分离；历史只在策略采样时推进，
不能把 1000 Hz 的硬件循环误当作 50 Hz 的模型历史。模型切换/停止/故障后的旧 epoch
结果不能重新接管关节。该实现没有宣称硬实时认证，必须在目标板测量控制、串口和策略的尾延迟。

### 模型约定

- 推荐模型 `policy_combined.onnx`：输入 `obs_history` 为 FP32 `[1,270]`，输出 `actions` 为 `[1,12]`。
- 6 帧包含当前帧，顺序为 `[当前, 前1帧, …, 前5帧]`。进入策略后首帧为真实观测，旧历史与前次动作清零。
- 编码器已集成；45 维已经包含 3 维 command，不能再次拼接。
- `policy.onnx` 是单独的 actor，输入 64 维，不适用于本控制器。
- 动作在模型关节顺序下解释为 `q_des = q_default + 0.25 × action`，训练侧再裁剪到 ±100 rad。
  控制器随后施加实际关节限位；训练侧的 ±100 不是机械限位。
- 训练 PD 是关节输出侧 `Kp=25 N·m/rad`、`Kd=0.5 N·m·s/rad`。这些参数需要真实执行器验证。
  SDK 层负责减速比/方向/零偏，不要在 RL 适配器重复转换。
- 命令训练范围为 `vx,vy ∈ [-1,1] m/s`、`yaw_rate ∈ [-2,2] rad/s`。

| 当前帧索引 | 数据 | 缩放 |
| --- | --- | --- |
| 0–2 | 机身/航向坐标系的 vx、vy、偏航角速度指令 | 1 |
| 3–5 | 机身角速度，rad/s | 0.25 |
| 6–8 | 世界单位重力 `[0,0,-1]` 旋转到机身坐标系 | 1 |
| 9–20 | 关节输出侧 `q - q_default`，rad，模型顺序 | 1 |
| 21–32 | 关节输出侧 dq，rad/s，模型顺序 | 0.05 |
| 33–44 | 上次采纳的原始网络动作，缩放与目标限位之前 | 1 |

各项先裁剪到 `[-100,100]` 再缩放。重力投影不是加速度计原始输出；不向真机观测添加训练噪声。
IMU 四元数使用 WXYZ，表示机身到世界的旋转，角速度必须在机身坐标系。
IMU 驱动自己的轴向转换与控制器安装旋转不能重复应用。
FDILink 驱动单独跟踪合法 AHRS 姿态的接收时刻，`orientation_timeout_s` 默认 0.1 秒。
姿态过期、载荷不完整或 CRC 无效时，后续陀螺帧不能刷新该姿态；没有可用的新鲜姿态时停止发布 `/imu`，
随后控制器的 IMU 看门狗退出策略。这不是硬件时间同步，实际传输延迟仍需在 NX 上测量。

模型关节顺序：`FL/FR/RL/RR hip`，然后同顺序 thigh，再 calf。
驱动顺序：`FR/FL/RR/RL`，每腿 hip/thigh/calf。
模型索引到驱动索引为 `[3,0,9,6,4,1,10,7,5,2,11,8]`，对应
`q_policy = q_sdk[mapping]`、`target_sdk[mapping] = target_policy`。

## NX 构建

目标是 **Jetson Orin NX（包括 Super 功耗模式）+ Ubuntu 22.04 + ROS 2 Humble**。
JetPack 6 系列提供 Ubuntu 22.04 基础系统；具体载板应使用厂商支持的 JetPack 镜像。
[NVIDIA Super 模式说明](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/)
及 [JetPack 系统说明](https://developer.nvidia.com/embedded/jetpack-sdk-622)。

本策略约 1 MB，先用 CPU 单线程 FP32 路径保持与桌面验证一致；是否需要 TensorRT 应由 NX 实测决定。
当前没有提供或验证 TensorRT 后端。不要复制桌面生成的 TensorRT engine 到 NX。
ONNX Runtime 的 [C++ SDK](https://onnxruntime.ai/docs/get-started/with-cpp) 与
[源代码构建说明](https://onnxruntime.ai/docs/build/inferencing.html) 可用于不同目标系统适配。

推荐两个仓库同级，先检出相应功能分支：

```bash
git clone --branch feat/him-rl-jetson-deployment https://github.com/hsc13576717115/Dog-control.git
git clone --branch feat/terrain-16099-deployment https://github.com/hsc13576717115/himloco_custom_dog.git
cd Dog-control
sudo apt update
sudo apt install build-essential cmake curl git-lfs python3-colcon-common-extensions \
  libssl-dev libeigen3-dev ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-realtime-tools ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-tf2-geometry-msgs
git -C ../himloco_custom_dog lfs install --local
git -C ../himloco_custom_dog lfs pull --include='deployment/terrain_16099/policy.pt'

export ONNXRUNTIME_ROOT="$(src/custom_dog_rl/scripts/install_onnxruntime.sh)"
src/custom_dog_rl/scripts/build_jetson.sh
```

该脚本只构建，不启动电机。SDK 固定为 ONNX Runtime 1.23.2 CPU，包含 x64/aarch64 下载校验。
不需要 PyTorch、Isaac Sim、CUDA 推理库或 OCS2。现有硬件包的 package.xml 仍保留 NMPC 依赖声明，
因此不要对整个工作区盲目运行 `rosdep install --from-paths src`；上面列出 RL 路径依赖，构建脚本显式选择包。
如使用其他 ROS 安装位置，设置 `CUSTOM_DOG_ROS_SETUP`；模型来源可用 `CUSTOM_DOG_DESCRIPTION_DIR` 指定；
执行器 SDK 可用 `UNITREE_ACTUATOR_SDK_ROOT` 指定。默认使用仓库内按架构选择的 Unitree SDK。

在新终端使用 `source install_rl/setup.bash`，不要混用已有 NMPC 构建的 overlay。
迁移时复制源码、模型及配置，重新在 NX 编译，不能复制 x86_64 的 `.so` 或 `build/`、`install/`。

## 离线与模拟硬件验证

在仓库根目录进行 C++ 离线测试，不安装 ROS 也可构建核心：

```bash
cmake -S src/custom_dog_rl -B /tmp/customdog-rl-core \
  -DCUSTOM_DOG_RL_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" \
  -DCUSTOM_DOG_RL_TEST_MODEL="$PWD/../himloco_custom_dog/deployment/terrain_16099/policy_combined.onnx"
cmake --build /tmp/customdog-rl-core -j2
ctest --test-dir /tmp/customdog-rl-core --output-on-failure
# 只依赖 NumPy；work-dir 每次选择新目录，避免覆盖先前结果。
python3 src/custom_dog_rl/tools/replay_validation.py \
  --probe /tmp/customdog-rl-core/policy_probe \
  --deployment-dir ../himloco_custom_dog/deployment/terrain_16099 \
  --work-dir /tmp/customdog-rl-replay --repeat 20
```

该回放会验证相关文件哈希、比较 200 组历史与原 checkpoint 的 golden 动作，并输出本机 C++ 推理的均值、
p95/p99/最大延迟。默认不需要 PyTorch，`--torchscript` 可额外验证 JIT。已安装 ROS 包时探针位于
`install_rl/custom_dog_rl/lib/custom_dog_rl/policy_probe`。
使用已安装的探针前需要 `source install_rl/setup.bash`，以加载同包共享库的搜索路径。

ROS 集成单元测试使用内存接口、合成 IMU 和真实 ONNX 模型，不连接执行器：

```bash
source /opt/ros/humble/setup.bash
cmake -S src/custom_dog_rl -B /tmp/customdog-rl-tests \
  -DCUSTOM_DOG_RL_BUILD_ROS=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" \
  -DCUSTOM_DOG_RL_TEST_MODEL="$PWD/../himloco_custom_dog/deployment/terrain_16099/policy_combined.onnx"
cmake --build /tmp/customdog-rl-tests -j2
ctest --test-dir /tmp/customdog-rl-tests --output-on-failure
```

测试 `controller_manager` / 插件加载 / ROS 话题连接：

```bash
source install_rl/setup.bash
ros2 launch custom_dog_rl mock.launch.py \
  model_path:="$PWD/../himloco_custom_dog/deployment/terrain_16099/policy_combined.onnx"
```

另一个加载同一环境的终端：

```bash
ros2 topic pub --once /rl_mock/rl_controller/mode std_msgs/msg/String '{data: shadow}'
ros2 topic echo /rl_mock/rl_controller/status
```

`GenericSystem` 是模拟硬件接口，并非动力学仿真，不能用这个画面/反馈判断步态能力。
它默认处于独立 `/rl_mock` 命名空间，合成 IMU 也在该命名空间，避免污染实机 `/imu`。
状态包含 mode、fault、actuation、inference_ms、observation_age_ms、inference_count。
桌面实测证据见 [C++ 与 ROS 验证记录](test/results/cpp-policy-replay.json)；不是 NX 性能数据。

FDILink 的伪串口回归直接驱动真实解析器，覆盖姿态停止、坏 CRC/帧尾、NaN、短载荷和恢复：

```bash
source install_rl/setup.bash
python3 src/fdilink_ahrs/test/test_orientation_freshness.py \
  --driver install_rl/fdilink_ahrs/lib/fdilink_ahrs/ahrs_driver_node --device-type 1
```

测试仅打开临时 PTY，不使用真实 IMU；`--device-type 0` 可验证原始坐标路径。

## 实机启动与状态机

先复制 [rl_controller.yaml](config/rl_controller.yaml) 为本机配置，核对 IMU 安装四元数、关节限位、增益和超时。
参数在 configure 阶段读取，修改配置后需要重新配置/启动控制器，运行中直接 `ros2 param set` 不会重新生成网络与控制参数。
推理线程默认单线程；`target_rate_limit_rad_s=0` 保持训练动作响应。开启目标限速会改变策略看到的执行器行为，应重新测试。
`stand_kp/stand_kd` 只控制起身与 ready，默认同样为 25/0.5，可在系留时独立调整以满足站姿稳定条件；
`kp/kd` 控制 RL 运行时的关节 PD，不要为解决起身下沉而同时改变策略运行增益。
若站姿误差一直未满足 `stand_tolerance_rad`，控制器会超时退出，应检查机械参数、站姿与实际承载，不能直接绕过 ready 门槛。

下列为接入时的命令模板，执行会打开真实串口；必须先替换端口与标定参数。
`physical_estop_verified:=true` 是完成现场急停核验后的声明，不能当作检测命令。

```bash
source install_rl/setup.bash
ros2 launch custom_dog_rl real.launch.py \
  model_path:="$PWD/../himloco_custom_dog/deployment/terrain_16099/policy_combined.onnx" \
  config_file:=/absolute/path/to/robot_rl.yaml \
  fr_port:=/dev/ttyS3 fl_port:=/dev/ttyS4 rr_port:=/dev/ttyS7 rl_port:=/dev/ttyS8 \
  imu_port:=/dev/ttyUSB0 \
  physical_estop_verified:=true enable_actuation:=false
```

默认 `enable_actuation=false`，拒绝起身和 RL 驱动请求，shadow 输出零位置刚度、零阻尼、零前馈。
串口仍会工作，这个开关不等于没有电机通信。完成反馈核对后，明确重新启动并设置 `enable_actuation:=true` 才能起身和行走。
同一时间只能加载 RL 或 NMPC 中的一个关节控制器。

| 指令：发布到 `/rl_controller/mode` | 作用与条件 |
| --- | --- |
| `calibrate` | 被动状态下，清除旧标定、等待反馈清除，再请求新标定；结束回到 passive |
| `stand` | 已标定、传感器健康、已使能运动时插值到模型默认站姿，稳定后进入 ready |
| `rl` | 仅从 ready 接管，启动新的历史/推理 epoch |
| `shadow` | 被动状态且反馈已标定时只推理；不输出电机驱动力 |
| `passive` | 停止策略；故障时必须健康条件恢复才复位，不会自动进入 RL |
| `estop` | 锁存故障，取消当前策略结果；真机运动使能时退回安全阻尼 |

例如发送 `ros2 topic pub --once /rl_controller/mode std_msgs/msg/String '{data: calibrate}'`，
观察 `/rl_controller/status` 回到 passive 后，再分别发送 stand、确认 ready、发送 rl。
速度指令入口是私有 `/rl_controller/cmd_vel`，需持续更新，例如：

```bash
ros2 topic pub --rate 20 /rl_controller/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.0}, angular: {z: 0.0}}'
```

命令超过 0.3 秒没有更新则将输入速度设为零，策略仍在 RL 模式维持平衡。
IMU/关节异常、温度/姿态超限、推理超时或标定丢失会锁存 fault，停止使用旧动作。
`/rl_controller/estop` 的 `std_msgs/Bool true` 同样锁存急停，false 只撤销输入电平，不能自动恢复运动。
关节侧 PD 的预测力矩超限时减小增益，实际电机仍需自己的硬件保护；该预测不是精确的电机力矩测量。

## 真机还需要落实的内容

这些是实物参数和现场验收，不是通过模型转换就能确定的数据：

1. NX/载板实际 JetPack、Ubuntu、串口设备与供电散热配置；板上推理和四路串口的尾延迟。
2. 12 个电机的 ID、腿序、正方向、减速比，以及已知折腿标定姿态。现有驱动的标定是记录当前姿态零偏，不会自动寻零。
3. IMU 安装朝向、驱动轴向、四元数与角速度单位、发布频率/时间戳；确认静止时机身重力为 `[0,0,-1]`。
4. 机械限位、允许力矩/温度、关节侧 PD 增益、软硬件急停与控制周期。软件守护无法代替切断电源的硬件急停。
5. 悬空核对每个关节方向和限位，再低速系留试验。记录实际观测/动作、串口失败与状态切换，最后逐步增加速度及地形难度。

当前硬件驱动中的 `physical_estop_verified` 是人工确认门槛，`physical_estop` 状态没有实际 GPIO 采样实现。
GPIO/继电器型号、有效电平与线路尚未知，需要按实物连接；不能把默认的 0 当成已具备硬件急停监测。
控制器提供软件急停入口供外部 GPIO 桥接，但电源切断必须独立于 Linux/ROS 进程。

不需要为该策略增加雷达、双目或接触传感器，输入来自现有关节与 IMU。
策略仅覆盖训练过的运动能力，不包含自主导航或摔倒后自恢复策略。
