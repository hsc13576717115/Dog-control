# Custom Dog Control Workspace

这是自定义四足机器人的 ROS 2 Humble 控制工作区，目标平台是 Ubuntu 22.04 和香橙派 5 Plus ARM64。控制主线使用传统控制，不使用强化学习：

```text
ros2_control + OCS2 NMPC + weighted WBC + Pinocchio state estimation
```

当前配置已通过平地起身、站立，以及 `vx +/-1.5 m/s`、`vy +/-1.0 m/s`、
`yaw +/-2.0 rad/s` 的 Gazebo Trot 速度包线。开发机运行 Gazebo，香橙派运行 IMU、
串口电机、估计器、NMPC、WBC 和诊断节点；仿真包线通过不代表真机可直接使用。

## 目录和命名

```text
Dog-control/
├── src/
│   ├── custom_dog_control/       # 控制器、NMPC、WBC、ros2_control 插件
│   ├── fdilink_ahrs/             # FDILink IMU 驱动
│   ├── serial_ros2/              # 串口基础库
│   └── unitree_guide/            # 历史参考代码
└── README.md
```

当前包名遵循 `robot_role` 的 snake_case 规则：

| 包 | 职责 |
| --- | --- |
| `custom_dog_control` | NMPC、WBC、状态机、估计器和 ros2_control 插件 |
| `fdilink_ahrs` | IMU 驱动和 `/imu` 发布 |
| `serial`（目录 `serial_ros2`） | 上游串口通信基础设施 |

模型包 `custom_dog_description` 位于同一 `custom_dog_stack` 的 ROS 2 underlay：
`/home/hsc/Dog_RL/custom_dog_stack/ros2/src/custom_dog_description`。构建本工作区前必须先构建并 source 该 underlay。

`qr_guide` 不再作为包名、插件名或运行时命名空间使用。迁移前版本由 Git 标签 `pre-ros2-nmpc-wbc` 保留，ROS 1 `unitree_guide` 通过 `COLCON_IGNORE` 排除在构建之外。

各参考工程的职责边界如下：

| 来源 | 本工程采用的内容 |
| --- | --- |
| `legged_control` | 固定提交 `a7f381c...` 的 `LeggedInterface` 与 `WeightedWbc` 算法源码 |
| `unitree_guide` | 操作习惯、状态切换和增量式键盘控制语义 |
| 原 `qr_guide` | GO-M8010-6/四路 RS485 通信、关节换算和 START 趴姿零点标定 |
| `custom_dog.urdf` + Pinocchio | 唯一运动学、动力学、坐标系和关节限位来源 |

旧 `qr_guide` 的 `RobotModel/LegKinematics` 不参与运行，避免把旧机械尺寸或旧零点混入新模型。

NMPC/WBC 不是仅按接口仿写：`qiayuanl/legged_control` 提交
`a7f381c0367e98e31c01336e678eef47e304d40d` 的 `legged_interface` 算法源码
直接编译为 `custom_dog_control_legged_interface`，WBC 直接编译其
`legged::WeightedWbc`。ROS 2 控制器只负责状态机、消息、实时缓冲和硬件适配。
上游文件及明确标注的本地 WBC 适配 SHA-256 记录在
`config/legged_control_upstream.sha256`，测试会阻止未登记的算法源码变化，也会阻止
后端退回 OCS2 示例接口。

## 控制架构

```text
/imu + /joy + /cmd_vel
          |
          v
  custom_dog_control controller
          |
  estimator + safety monitor
       |             |
       |        FAULT / damping
       v
  OCS2 NMPC (50 Hz) ---> realtime policy buffer ---> weighted WBC (1000 Hz)
                                                            |
                                  GazeboSystem / UnitreeSystemInterface
```

状态机固定为：

```text
PASSIVE -> CALIBRATION -> STAND_UP -> MPC_STANCE <-> MPC_TROT
   ^                                                |
   +---------------------- FAULT <-----------------+
```

## 模型约定

唯一动力学模型为 `custom_dog_description/urdf/custom_dog.urdf`。关节顺序固定为 `FR、FL、RR、RL`，每条腿为 `hip、thigh、calf`；接触帧为 `FR_foot、FL_foot、RR_foot、RL_foot`。内部坐标遵循 REP-103：x 向前、y 向左、z 向上。CAD 总质量为 `13.84916 kg`。控制启动会检查质量、惯量、关节限位、足端名称和 Pinocchio FK/Jacobian。

## 依赖准备

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions python3-vcstool \
  libeigen3-dev libyaml-cpp-dev libtinyxml-dev \
  liburdfdom-dev \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-urdf ros-humble-urdfdom \
  ros-humble-pinocchio ros-humble-coal \
  ros-humble-xacro ros-humble-rviz2
```

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
source /opt/ros/humble/setup.bash
git submodule update --init --recursive
export CUSTOM_DOG_CONTROL_DEPS_WS=/home/hsc/Dog_RL/custom_dog_stack/third_party/custom_dog_control_deps_ws
src/custom_dog_control/scripts/fetch_ocs2.sh "$CUSTOM_DOG_CONTROL_DEPS_WS"
cd "$CUSTOM_DOG_CONTROL_DEPS_WS"
export LIBRARY_PATH="/opt/ros/humble/lib/$(gcc -print-multiarch):${LIBRARY_PATH:-}"
colcon build --symlink-install \
  --packages-up-to ocs2_legged_robot ocs2_self_collision ocs2_sqp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys="grid_map_filters_rsl convex_plane_decomposition_ros"
```

版本锁定在 `src/custom_dog_control/config/dependencies.lock.yaml`。

首次构建前先准备模型 underlay：

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_dog_description
source install/setup.bash
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
```

## 开发机仿真

推荐使用带前置检查的一键入口。它会在本工作区构建前自动构建缺失的
`custom_dog_description` model underlay，并且不会在构建失败后继续启动旧的安装结果：

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
source /opt/ros/humble/setup.bash
src/custom_dog_control/scripts/build_simulation.sh
```

也可以分步执行，但必须先成功构建模型工作区，并在控制工作区构建成功后再
`source install/setup.bash`：

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/ros2
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select custom_dog_description
source install/setup.bash

cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
colcon build --symlink-install --packages-up-to custom_dog_control \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
               -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=OFF
source install/setup.bash
ros2 launch custom_dog_control gazebo.launch.py use_rviz:=false
```

仿真 Launch 默认在控制器成功激活后直接启动键盘控制，因此使用一键入口时不需要
第二个终端：

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
source /opt/ros/humble/setup.bash
src/custom_dog_control/scripts/build_simulation.sh
```

键盘输入由运行 Launch 的终端读取。无交互终端、自动测试或需要单独运行键盘节点时，
传入 `start_keyboard:=false`；随后可在另一个终端运行
`ros2 run custom_dog_control keyboard_teleop.py`。

键位沿用 `unitree_guide` 的布局：`1` 进入 PASSIVE，`2` 执行 START 标定并
起身。起身完成后自动进入可行走状态；非零速度指令自动进入 MPC_TROT，速度清零
后自动回到 MPC_STANCE 保持站立。`W/S`、`A/D`、`J/L` 分别增减前后、左右和偏航目标；
每次按键改变键盘限值的 5%，目标会持续生效。键盘与 `/cmd_vel` 使用相同包线：
`vx +/-1.5 m/s`、`vy +/-1.0 m/s`、`yaw +/-2.0 rad/s`。空格或 `X` 将全部速度清零，
控制器会先按限加速度减速，再切回 MPC_STANCE；`Esc` 触发软件 FAULT 急停
（物理急停仍必须独立存在）。`Q/E` 保留为 `J/L` 的偏航别名。

Gazebo 使用与真机相同的控制器，只把 `q/qd/kp/kd/tau` 换算成仿真关节力矩。建议先用仿真真值验证站立和步态，再切换到实际估计器。
位置控制只用于 `STAND_UP`：控制器从测得的趴姿插值到站立关节角，并平滑交接。
进入 `MPC_STANCE` 或 `MPC_TROT` 后，Gazebo 和真机都执行同一套 OCS2 NMPC
策略与 weighted WBC，仿真不存在位置控制步态或运动兜底。Gazebo 足端接触参数采用
显式 ODE 摩擦、刚度和阻尼配置，以便稳定复现实机接触约束。
Trot 使用固定 `0.25 s` 周期和 50% 占空比的对角接触序列；零速不原地踏步，
而是保持四足 STANCE。这里借鉴 `unitree_guide` 的固定步频、相位安全切换和
“目标速度不得远离实测速度”的设计，但不叠加其解析逆运动学摆腿轨迹，避免与
OCS2 的 NMPC 状态、接触约束和 WBC 任务产生两套相互冲突的足端目标。
仿真会暂停物理并按趴下标定角生成模型，等两个控制器均激活后再自动恢复物理。
默认模式仍是 `PASSIVE`。Gazebo 在该模式使用可配置的初始姿态保持，避免重力在
控制准备阶段将关节拉到限位；真机明确关闭此功能并保持 `Kp=0、Kd=1.0`。
趴下与校准姿态的 URDF 名义值统一为每腿
`hip=0 deg、thigh=71.8 deg、calf=-161.8 deg`。旧 `qr_guide` 的镜像运动学
零点不进入 NMPC/WBC；运行时 FK、Jacobian 和动力学统一由 URDF Pinocchio 模型提供。
真机上，操作者先手动把四条腿放到该趴下姿态，再按手柄 START。START 只表达
“当前机械姿态已经是上述名义 URDF 角度”：硬件层读取本周期全部 12 个有效反馈，计算
每个电机的零点偏差，随后反馈和指令都直接使用 URDF 关节坐标。任一电机本周期
通信失败时不会完成标定。
该角度仍是手工折叠姿态的名义值，最终可使用实机机械基准或标定工装测得的角度
覆盖，不能把手工目测姿态当作高精度绝对零点。
贴地几何只能约束 thigh/calf；hip 的 `0 deg` 仍是左右对称的设计名义值，需要用
实际髋关节基准或工装确认。
动力学碰撞在承载接触的机身、髋和足端使用 box/cylinder/sphere 简化几何，
CAD STL 只用于显示。大腿和小腿当前不添加未经 CAD 包络校核的碰撞体，避免
虚假自碰撞改变已验收的接触动力学。

## 香橙派真机

在香橙派上重新编译 ARM64 版本，不要复制开发机生成的 OCS2 CppAD 缓存：

```bash
source /opt/ros/humble/setup.bash
export UNITREE_ACTUATOR_SDK_ROOT=/absolute/path/to/unitree_actuator_sdk
colcon build --packages-up-to custom_dog_control \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=ON \
               -DUNITREE_ACTUATOR_SDK_ROOT="$UNITREE_ACTUATOR_SDK_ROOT"
```

真实启动默认禁止激活：

```bash
source install/setup.bash
ros2 launch custom_dog_control real.launch.py \
  physical_estop_verified:=true \
  calibration_hip_deg:=0.0 \
  calibration_thigh_deg:=71.8 \
  calibration_calf_deg:=-161.8
```

只有独立物理急停完成安装和验收后才允许使用该参数。
三个 `calibration_*_deg` 参数是 START 标定时当前折叠姿态对应的 URDF 角度，后续
实测值可直接在 launch 命令中覆盖，不需要重新编译。
真机覆盖配置当前与仿真一致，为 `vx +/-1.5 m/s`、`vy +/-1.0 m/s`、
`yaw +/-2.0 rad/s`。这只是指令限幅配置，不代表真机已经通过高速验收；启用前仍必须
完成独立物理急停、系留和分阶段低速测试。

## 接口和测试

输入为 `/imu` (`sensor_msgs/msg/Imu`)、`/joy` (`sensor_msgs/msg/Joy`) 和 `/cmd_vel`
(`geometry_msgs/msg/Twist`)。仿真速度限幅为 `vx +/-1.5 m/s`、`vy +/-1.0 m/s`、
`yaw +/-2.0 rad/s`。输出包括 `/joint_states`、`/odom`、TF、控制模式、接触计划、
NMPC 状态、WBC 残差和 RS485 诊断。

```bash
colcon test --packages-select custom_dog_control
colcon test-result --verbose
python3 src/custom_dog_control/test/check_urdf_contract.py \
  "$(ros2 pkg prefix --share custom_dog_description)/urdf/custom_dog.urdf"
```

Gazebo 启动后，可在另一个已 source 相同工作区的终端运行自动验收：

```bash
ros2 run custom_dog_control simulation_smoke_test.py
ros2 run custom_dog_control simulation_motion_test.py
ros2 run custom_dog_control simulation_command_matrix_test.py
ros2 run custom_dog_control simulation_envelope_test.py --vx 1.5
ros2 run custom_dog_control simulation_envelope_test.py --vx -1.5
ros2 run custom_dog_control simulation_envelope_test.py --vy 1.0
ros2 run custom_dog_control simulation_envelope_test.py --vy -1.0
ros2 run custom_dog_control simulation_envelope_test.py --yaw 2.0
ros2 run custom_dog_control simulation_envelope_test.py --yaw -2.0
```

`simulation_motion_test.py` 会从 PASSIVE 起身，切换到 NMPC/WBC Trot，以
`0.05 m/s` 前进 12 秒，再清零速度并要求回到 NMPC_STANCE；过程中会检查 FAULT、
WBC 有效性、零速启用 Trot 时保持 STANCE、前向位移、侧偏、航向、姿态、髋内收和
停车速度。`simulation_command_matrix_test.py` 从独立冷启动依次验收前进、侧移和
原地转向，每段都必须从 Trot 正常停止到 STANCE。测试还要求运行时诊断明确
报告 `legged::LeggedInterface`、`legged::WeightedWbc` 和固定上游提交。运行自动测试时应以
`start_keyboard:=false` 启动 Gazebo，避免键盘节点同时发布命令。
`simulation_envelope_test.py` 每次只验证一个速度轴，执行起身前站立检查、20 秒满量程
运动、受控停车和 10 秒停车后站立保持，并统计末段速度、姿态 RMS/峰值、机身高度及
NMPC/WBC 有效性。六个正负方向必须各自在全新 Gazebo 世界运行，不能复用已经积累
位姿误差或接触状态的世界来替代独立验收。

当前固定 `0.25 s` 周期配置的独立冷启动验收结果如下；线速度允许误差为 `0.20 m/s`，
角速度允许误差为 `0.30 rad/s`，六项均完成受控停车及 10 秒 MPC_STANCE 保持：

| 目标 | 稳态实测 | roll/pitch RMS | 最大高度误差 |
| --- | --- | --- | --- |
| `vx +1.5 m/s` | `+1.472 m/s` | `0.005 / 0.008 rad` | `0.026 m` |
| `vx -1.5 m/s` | `-1.475 m/s` | `0.003 / 0.009 rad` | `0.025 m` |
| `vy +1.0 m/s` | `+1.000 m/s` | `0.018 / 0.008 rad` | `0.016 m` |
| `vy -1.0 m/s` | `-0.994 m/s` | `0.019 / 0.007 rad` | `0.016 m` |
| `yaw +2.0 rad/s` | `+2.008 rad/s` | `0.002 / 0.004 rad` | `0.008 m` |
| `yaw -2.0 rad/s` | `-2.007 rad/s` | `0.002 / 0.003 rad` | `0.008 m` |

真机顺序固定为：物理急停、单电机、单腿悬空、四腿悬空、位置起身、悬挂 WBC、绳保护站立、低速前后/侧移/转向。任一通信超时、温升异常、方向错误、求解失败或振荡都必须停止测试。

更多控制器细节见 [src/custom_dog_control/README.md](src/custom_dog_control/README.md)。
