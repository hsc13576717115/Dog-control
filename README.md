# Custom Dog Control

面向自制四足机器人的 ROS 2 Humble 传统运动控制工程，目标平台为 Ubuntu 22.04、
ARM64 香橙派 5 Plus。项目不使用强化学习，控制主线为：

```text
ros2_control + OCS2 NMPC + weighted WBC + Pinocchio
```

仿真与真机共用同一个控制器。位置插值只负责从趴姿起身，站立和 Trot 均由
NMPC-WBC 控制。

## 当前状态

| 项目 | 状态 |
| --- | --- |
| Gazebo 起身、站立、Trot | 已跑通 |
| 固定 Trot 周期 | `0.25 s`，占空比 `0.5` |
| 控制器 / NMPC 频率 | `1000 Hz / 50 Hz` |
| Gazebo 六方向速度包线 | `vx +/-1.5 m/s`、`vy +/-1.0 m/s`、`yaw +/-2.0 rad/s` 已通过 |
| Gazebo 地形基线 | 6 项中 5 项通过；10° 坡包在坡顶停滞 |
| 香橙派 ARM64 构建入口 | 已提供 |
| 实机通信、估计器和落地行走 | 尚未完成分阶段验收 |

> **安全说明：** 当前版本可以开始悬空和系留条件下的实机调试，但不能直接用于
> 无保护落地或高速行走。Gazebo 速度包线使用仿真基座真值；真机状态估计、12 电机
> RS485 周期和物理急停必须分别验收。

## 快速启动仿真

推荐使用一键脚本。脚本会构建模型 underlay、控制工作区并启动 Gazebo：

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
source /opt/ros/humble/setup.bash
src/custom_dog_control/scripts/build_simulation.sh
```

已经完成构建时，可直接启动：

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
source /opt/ros/humble/setup.bash
source /home/hsc/Dog_RL/custom_dog_stack/ros2/install/setup.bash
source /home/hsc/Dog_RL/custom_dog_stack/third_party/custom_dog_control_deps_ws/install/setup.bash
source install/setup.bash
ros2 launch custom_dog_control gazebo.launch.py use_rviz:=false
```

Launch 默认在控制器激活后启动键盘节点，按键应发送到启动 Launch 的终端。

### 键盘控制

| 按键 | 功能 |
| --- | --- |
| `1` | 进入 PASSIVE；FAULT 后用于确认复位 |
| `2` | 标定当前趴姿、插值起身并使能 NMPC-WBC |
| `W / S` | 增加 / 减少前向速度 |
| `A / D` | 增加 / 减少侧向速度 |
| `J / L` | 增加 / 减少偏航角速度 |
| `Q / E` | `J / L` 的兼容别名 |
| `Space` 或 `X` | 速度平滑清零并回到 MPC_STANCE |
| `Esc` | 触发软件 FAULT 急停 |

每次速度按键改变对应上限的 5%。起身后，非零速度会自动进入 MPC_TROT；速度归零后
自动回到四足 MPC_STANCE，不需要分别操作 `3` 和 `4`。

需要单独运行键盘节点时：

```bash
# 终端 1
ros2 launch custom_dog_control gazebo.launch.py \
  use_rviz:=false start_keyboard:=false

# 终端 2（source 相同的四层环境后）
ros2 run custom_dog_control keyboard_teleop.py
```

## 控制架构

```text
 /imu       /joy       /cmd_vel
    \         |          /
     +--------+---------+
              |
              v
     NmpcWbcController (1000 Hz)
       |       |         |
       |       |         +--> safety monitor --> FAULT / damping
       |       +------------> state estimator
       +--------------------> OCS2 NMPC (50 Hz)
                                  |
                           realtime policy buffer
                                  |
                                  v
                         weighted WBC (1000 Hz)
                                  |
                    +-------------+-------------+
                    |                           |
             Gazebo joint effort       UnitreeSystemInterface
```

状态机固定为：

```text
PASSIVE -> CALIBRATION -> STAND_UP -> MPC_STANCE <-> MPC_TROT
   ^                                               |
   +--------------------- FAULT <-----------------+
```

- `STAND_UP` 从实际关节反馈插值到站立姿态，并平滑交接 WBC。
- `MPC_STANCE` 使用四足接触计划保持站立。
- `MPC_TROT` 使用 `FR+RL` 和 `FL+RR` 两组对角接触。
- 真机 NMPC 策略超过两个求解周期未更新时进入 FAULT；Gazebo 为适应桌面非实时调度，
  默认放宽到四个周期。硬件、IMU 或限位检查失败同样进入 FAULT。
- Gazebo 与真机没有位置控制步态或其他运动控制兜底。

## 模型与标定约定

唯一运动学和动力学模型为：

```text
/home/hsc/Dog_RL/custom_dog_stack/ros2/src/custom_dog_description/urdf/custom_dog.urdf
```

| 项目 | 约定 |
| --- | --- |
| 总质量 | CAD 值 `13.84916 kg` |
| 坐标系 | REP-103：x 向前、y 向左、z 向上 |
| 腿顺序 | `FR、FL、RR、RL` |
| 单腿关节顺序 | `hip、thigh、calf` |
| 接触帧 | `FR_foot、FL_foot、RR_foot、RL_foot` |
| 名义趴姿 | 每腿 `hip=0 deg、thigh=71.8 deg、calf=-161.8 deg` |

运行时 FK、Jacobian、惯量和关节限位均来自 URDF Pinocchio 模型。原 `qr_guide`
的 `RobotModel/LegKinematics` 已退出运行路径，避免旧机械尺寸和旧运动学零点污染模型。

真机按下 START 前，操作者需要将四条腿置于名义趴姿。START 表示“当前机械姿态对应
上述 URDF 角度”：硬件层仅在 12 个电机反馈全部有效时计算零点偏移。该角度是当前
人工折叠姿态的名义值，后续应通过机械基准或标定工装修正；尤其是 hip 的 `0 deg`
不能由贴地几何单独确定。

Gazebo 中看到的趴姿是碰撞和重力稳定后的生成姿态，髋关节会有左右对称的外展；它由
仿真专用 `passive_joint_positions` 描述，不等同于真机 START 建立的逻辑标定角。
二者最终都转换到同一套 URDF 关节坐标后再进入 NMPC-WBC。

## 工程结构与参考边界

```text
Dog-control/
├── src/
│   ├── custom_dog_control/   # 控制器、NMPC、WBC、估计器和硬件插件
│   ├── fdilink_ahrs/         # FDILink IMU 驱动
│   ├── serial_ros2/          # 串口基础库
│   └── unitree_guide/        # ROS 1 历史参考，已由 COLCON_IGNORE 排除
└── README.md
```

| 来源 | 本工程采用的内容 |
| --- | --- |
| `qiayuanl/legged_control` | 固定提交的 `LeggedInterface`、OCS2 质心模型和 `WeightedWbc` |
| `unitree_guide` | 状态组织、操作语义、增量键盘和固定步态思路 |
| 原 `qr_guide` | GO-M8010-6、四路 RS485、关节换算和趴姿零点标定 |
| `custom_dog.urdf` + Pinocchio | 唯一模型、坐标、关节限位和运动学来源 |

ROS 2 控制器负责状态机、消息、实时数据交换和硬件适配。工程直接编译
`legged_control` 提交 `a7f381c0367e98e31c01336e678eef47e304d40d` 的算法源码，
本地 WBC 适配包括预测关节限位和基座反馈。

## 安装与构建

### 1. 系统依赖

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions python3-vcstool \
  libeigen3-dev libyaml-cpp-dev libtinyxml-dev liburdfdom-dev \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-urdf ros-humble-urdfdom ros-humble-xacro \
  ros-humble-pinocchio ros-humble-coal ros-humble-rviz2
```

### 2. OCS2 依赖工作区

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
```

依赖版本记录在
[`dependencies.lock.yaml`](src/custom_dog_control/config/dependencies.lock.yaml)。

### 3. 模型 underlay

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/ros2
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select custom_dog_description
```

### 4. 仿真版本

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
source /opt/ros/humble/setup.bash
source /home/hsc/Dog_RL/custom_dog_stack/ros2/install/setup.bash
source /home/hsc/Dog_RL/custom_dog_stack/third_party/custom_dog_control_deps_ws/install/setup.bash

rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys="grid_map_filters_rsl convex_plane_decomposition_ros"

colcon build --symlink-install --packages-up-to custom_dog_control \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=OFF
```

## 仿真验收

自动测试时关闭 GUI、RViz 和键盘节点：

```bash
ros2 launch custom_dog_control gazebo.launch.py \
  gui:=false use_rviz:=false start_keyboard:=false
```

在另一个已 source 相同环境的终端运行：

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

每项速度包线都必须使用全新 Gazebo 世界，依次完成起身、20 秒满量程运动、受控停车
和 10 秒 MPC_STANCE 保持。当前固定 `0.25 s` Trot 周期的独立冷启动结果为：

| 目标 | 稳态实测 | roll / pitch RMS | 最大高度误差 |
| --- | --- | --- | --- |
| `vx +1.5 m/s` | `+1.472 m/s` | `0.005 / 0.008 rad` | `0.026 m` |
| `vx -1.5 m/s` | `-1.475 m/s` | `0.003 / 0.009 rad` | `0.025 m` |
| `vy +1.0 m/s` | `+1.000 m/s` | `0.018 / 0.008 rad` | `0.016 m` |
| `vy -1.0 m/s` | `-0.994 m/s` | `0.019 / 0.007 rad` | `0.016 m` |
| `yaw +2.0 rad/s` | `+2.008 rad/s` | `0.002 / 0.004 rad` | `0.008 m` |
| `yaw -2.0 rad/s` | `-2.007 rad/s` | `0.002 / 0.003 rad` | `0.008 m` |

线速度容差为 `0.20 m/s`，角速度容差为 `0.30 rad/s`。这些数据是在
`use_sim_ground_truth: true` 下得到的，证明当前控制与接触模型在 Gazebo 中可运行，
不能代替真机估计器和硬件实时性验收。

### 地形通过测试

`world` Launch 参数可选择独立地形世界。每项测试都从原点冷启动，不要自行改变出生
坐标把机器人直接放到障碍附近，因为当前 NMPC 参考初始化假定原点出生。
以 5 cm 台阶为例：

```bash
# 终端 1
WORLD_DIR="$(ros2 pkg prefix --share custom_dog_control)/worlds"
ros2 launch custom_dog_control gazebo.launch.py \
  gui:=false use_rviz:=false start_keyboard:=false \
  world:="$WORLD_DIR/step_50mm.world"

# 终端 2
ros2 run custom_dog_control simulation_terrain_test.py \
  --name step_50mm --speed 0.25 --target-distance 3.2
```

测试节点要求起身交接后连续稳定站立 1 秒，再以 `0.25 m/s` 前进；通过条件为完成目标
距离、NMPC/WBC 全程有效、roll/pitch 不超过 `0.45 rad`、横漂不超过 `0.35 m`、
基座高度不低于 `0.12 m`，并能停车恢复 MPC_STANCE。独立冷启动结果如下：

| 世界 | 完成距离 | 最大 roll / pitch | 最大横漂 | 基座高度范围 | 结果 |
| --- | --- | --- | --- | --- | --- |
| `step_30mm.world` | `3.200 / 3.2 m` | `0.055 / 0.065 rad` | `0.013 m` | `0.281-0.315 m` | 通过 |
| `step_50mm.world` | `3.200 / 3.2 m` | `0.087 / 0.113 rad` | `0.025 m` | `0.279-0.355 m` | 通过 |
| `ramp_5deg.world` | `4.200 / 4.2 m` | `0.011 / 0.026 rad` | `0.010 m` | `0.283-0.324 m` | 通过 |
| `ramp_10deg.world` | `2.485 / 4.2 m` | `0.071 / 0.037 rad` | `0.040 m` | `0.282-0.415 m` | 失败：坡顶停滞 |
| `uneven_10_40mm.world` | `4.001 / 4.0 m` | `0.058 / 0.057 rad` | `0.013 m` | `0.282-0.312 m` | 通过 |
| `low_friction_mu_020.world` | `4.200 / 4.2 m` | `0.009 / 0.010 rad` | `0.011 m` | `0.282-0.289 m` | 通过 |

当前结果表示平地控制器具有一定被动越障余量，不表示已经实现地形自适应。现有
`SwitchedModelReferenceManager` 仍把摆动足地面高度固定为 `0.0 m`，没有高程图、
落脚点重规划或真实触地修正。10° 坡顶停滞与该限制一致；继续提高坡度、台阶高度或
速度前，应先把地形高度和法向量接入 NMPC 参考、摆动足轨迹与 WBC 接触任务。
`mu=0.20` 结果仅验证 `0.25 m/s` 直行通过，不构成完整的低摩擦速度包线。

## ROS 2 接口

### 输入

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/imu` | `sensor_msgs/msg/Imu` | 姿态与角速度测量 |
| `/joy` | `sensor_msgs/msg/Joy` | 手柄状态和速度输入 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 上层速度指令 |

非零手柄速度有效时优先于 `/cmd_vel`，两者共用速度和加速度限幅器；仅按模式键且摇杆
保持零位时，不会遮蔽 `/cmd_vel`。

### 输出

- `/joint_states`
- `/odom` 和 TF
- `/nmpc_wbc_controller/control_mode`
- `/nmpc_wbc_controller/contact_plan`
- `/nmpc_wbc_controller/diagnostics`

诊断数据包含 NMPC 状态、WBC 残差、控制周期、求解时间和硬件错误计数。

## 香橙派与真机

香橙派必须在目标 ARM64 系统重新构建，不要复制 x86 开发机生成的库或 OCS2 CppAD
缓存：

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
source /opt/ros/humble/setup.bash
source /home/hsc/Dog_RL/custom_dog_stack/ros2/install/setup.bash
source /home/hsc/Dog_RL/custom_dog_stack/third_party/custom_dog_control_deps_ws/install/setup.bash

export UNITREE_ACTUATOR_SDK_ROOT=/absolute/path/to/unitree_actuator_sdk
colcon build --packages-up-to custom_dog_control \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=ON \
  -DUNITREE_ACTUATOR_SDK_ROOT="$UNITREE_ACTUATOR_SDK_ROOT"
```

只有独立物理急停完成安装和验收后，才允许启动真实硬件：

```bash
source install/setup.bash
ros2 launch custom_dog_control real.launch.py \
  physical_estop_verified:=true \
  calibration_hip_deg:=0.0 \
  calibration_thigh_deg:=71.8 \
  calibration_calf_deg:=-161.8
```

`physical_estop_verified:=true` 只是操作者确认，不会替代硬件急停。物理急停必须独立于
ROS、香橙派和控制进程，能直接撤销执行器使能或动力电源。

真机配置当前保留与仿真相同的最大指令包线，但这只是限幅值，不代表已经通过高速
验收。首轮测试必须限制为低速，并严格按以下顺序推进：

1. 验证物理急停能够独立断开执行器。
2. 单电机检查 ID、方向、减速比、温度和力矩限制。
3. 单腿悬空完成标定、位置插值和安全阻尼测试。
4. 四腿悬空检查 12 电机同步、故障锁存和 WBC 输出方向。
5. 测量完整 RS485 回路的平均、P95、P99 周期和超时率。
6. 使用 `use_sim_ground_truth: false` 验证 IMU、编码器和接触计划估计器。
7. 悬挂完成 WBC 站立，再在安全绳保护下进行落地站立和低速六方向测试。
8. 只有连续测试无超时、温升、限位、求解失败或振荡时，才逐级扩大速度。

当前 1000 Hz 控制周期的名义时间为 `1.0 ms`。在香橙派和完整 12 电机通信链路上，
控制周期 P99 应不超过 `1.2 ms`，并且不能产生新的 IO 超时：

```bash
ros2 run custom_dog_control profile_runtime.py --ros-args \
  -p duration_s:=1800.0 \
  -p control_rate_hz:=1000.0 \
  -p output_csv:=/tmp/custom_dog_control_profile.csv
```

真机估计器使用 IMU、编码器、Pinocchio 运动学和计划接触相位。第一版没有足端传感器，
也没有基于真实触地信号的相位修正，这是仿真迁移到真机时的主要剩余风险之一。

## 单元测试

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
source /opt/ros/humble/setup.bash
source /home/hsc/Dog_RL/custom_dog_stack/ros2/install/setup.bash
source /home/hsc/Dog_RL/custom_dog_stack/third_party/custom_dog_control_deps_ws/install/setup.bash
source install/setup.bash

colcon test --packages-select custom_dog_control --event-handlers console_direct+
colcon test-result --verbose
```

测试覆盖模型契约、关节与足端顺序、REP-103 坐标转换、标定姿态、步态相位、速度
限幅、估计器姿态、安全锁存、关节限制、WBC 约束和上游算法来源检查。

## 版本与许可证

- 迁移前代码保存在 Git 标签 `pre-ros2-nmpc-wbc`。
- 第三方依赖固定版本见
  [`dependencies.lock.yaml`](src/custom_dog_control/config/dependencies.lock.yaml)。
- `legged_control` 上游文件及本地适配哈希见
  [`legged_control_upstream.sha256`](src/custom_dog_control/config/legged_control_upstream.sha256)。
- 包许可证为 BSD-3-Clause；引入的 `legged_control` 源码遵循 LGPL-2.1-only。
- 控制器内部实现说明见
  [`src/custom_dog_control/README.md`](src/custom_dog_control/README.md)。
