# custom_dog_control

`custom_dog_control` 是自定义四足机器人的 ROS 2 Humble 控制包。它使用同一个 `ControllerInterface` 驱动 Gazebo 和真实硬件，launch 文件只负责切换 ros2_control 后端。

## 模块

```text
include/custom_dog_control/          公共 C++ 接口
src/controller/                      NmpcWbcController 生命周期和状态机
src/nmpc/                            OCS2 NMPC、模型验证和状态估计
src/safety/                          超时、限位、温度和故障降级
src/hardware/                        ros2_control 实机插件与 GO-M8010-6 通信
config/                              控制器、NMPC、步态和硬件参数
launch/                              Gazebo 与真机启动入口
third_party/                         legged_control 算法源码和 qpOASES
```

迁移前的 qr_guide FSM、VMC/QP 和解析腿运动学已从本包删除。运行时运动学和
动力学全部来自 URDF 创建的 Pinocchio 模型；`hardware/IOSDK` 只负责串口、减速比、
电机方向和零点偏差，不包含足端正逆解。

NMPC 直接编译并实例化 `qiayuanl/legged_control` 固定提交 `a7f381c...` 的
`legged::LeggedInterface`，WBC 直接使用同一提交的 `legged::WeightedWbc`。
上游源码哈希由 `test_legged_control_provenance` 校验；ROS 2 适配没有用
`ocs2::legged_robot::LeggedRobotInterface` 替代该算法接口。

## 运行时

控制器插件：`custom_dog_control/NmpcWbcController`

硬件插件：`custom_dog_control/UnitreeSystemInterface`

默认频率：

| 环节 | 频率 |
| --- | --- |
| ros2_control / WBC | 1000 Hz |
| OCS2 NMPC | 50 Hz |
| Gazebo IMU | 250 Hz |

状态机为 `PASSIVE -> CALIBRATION -> STAND_UP -> MPC_STANCE <-> MPC_TROT`，所有严重故障进入锁存的 `FAULT`。

`STAND_UP` 使用关节位置插值并平滑交接 WBC。`MPC_STANCE` 和 `MPC_TROT`
在 Gazebo 与真机上都使用相同的 OCS2 NMPC + weighted WBC 控制链，不提供
仿真专用的位置控制运动模式。

## 编译与启动

仿真：

```bash
cd /home/hsc/Dog_RL/custom_dog_stack/third_party/Dog-control
source /opt/ros/humble/setup.bash
src/custom_dog_control/scripts/build_simulation.sh
```

该脚本使用 `set -euo pipefail`：模型 underlay、ROS 依赖或本工作区构建任一失败时
都会停止，不会继续使用残缺的 `install` 目录启动 Gazebo。

仿真 Launch 默认在控制器成功激活后直接启动键盘控制：

```bash
ros2 launch custom_dog_control gazebo.launch.py
```

键盘输入由运行 Launch 的终端读取。无交互终端、自动测试或需要单独运行键盘时，
使用 `start_keyboard:=false`，再从另一个终端执行
`ros2 run custom_dog_control keyboard_teleop.py`。

键位沿用 `unitree_guide` 的交互语义：`1` PASSIVE，`2` START 标定并起身。
起身完成后自动进入可行走状态；非零速度指令自动请求 TROT，速度清零后自动回到
`MPC_STANCE` 保持站立。`W/S`、`A/D`、`J/L` 以键盘限值的 5% 分别调整前后、左右和
偏航目标。键盘和 `/cmd_vel` 的仿真包线均为 `vx +/-1.5 m/s`、`vy +/-1.0 m/s`、
`yaw +/-2.0 rad/s`；空格或 `X` 触发受限减速并回到 STANCE，`Esc` 触发软件 FAULT 急停。
`Q/E` 保留为 `J/L` 的偏航别名。

动态步态固定使用 `0.20 s` 周期、50% 占空比的对角 Trot。控制器不在 NMPC
运行过程中切换步频；零速由四足接触的 MPC_STANCE 处理。

真机：

```bash
export UNITREE_ACTUATOR_SDK_ROOT=/absolute/path/to/unitree_actuator_sdk
colcon build --packages-up-to custom_dog_control \
  --cmake-args -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=ON \
               -DUNITREE_ACTUATOR_SDK_ROOT="$UNITREE_ACTUATOR_SDK_ROOT"
source install/setup.bash
ros2 launch custom_dog_control real.launch.py physical_estop_verified:=true
```

`physical_estop_verified` 只能在独立硬件急停完成验收后设置为 `true`。
折叠姿态有更准确的测量值时，通过 `calibration_hip_deg`、
`calibration_thigh_deg` 和 `calibration_calf_deg` 覆盖默认值。
真机覆盖配置当前与仿真一致，为 `vx +/-1.5 m/s`、`vy +/-1.0 m/s` 和
`yaw +/-2.0 rad/s`。这只是指令限幅配置，真机启用前仍必须完成独立物理急停、系留
和分阶段低速测试。

## 模型与接口

动力学和几何只来自 `custom_dog_description/urdf/custom_dog.urdf`。本包 NMPC 配置不重复保存质量、惯量、腿长或关节限位。

真机标定约定：按 START 前，四条腿必须已经处于手工折叠趴下姿态。仿真和真机
统一采用 `hip=0 deg、thigh=71.8 deg、calf=-161.8 deg`。START 将本周期 12 个有效电机反馈
对齐到配置的名义角度；它不是“回到电机零位”命令，也不会调用 qr_guide 运动学。

订阅：`/imu`、`/joy`、`/cmd_vel`。

发布：`/joint_states`、`/odom`、TF、控制模式、接触计划和诊断。

## 测试

```bash
colcon test --packages-select custom_dog_control
colcon test-result --verbose
```

无键盘启动 Gazebo 后可执行完整运动验收：

```bash
ros2 run custom_dog_control simulation_motion_test.py
ros2 run custom_dog_control simulation_command_matrix_test.py
ros2 run custom_dog_control simulation_envelope_test.py --vy -1.0
```

测试流程为位置插值起身、NMPC/WBC Trot 前进 12 秒、速度清零并回到
NMPC_STANCE，同时验证零速 Trot 门控、WBC、位移、姿态、髋内收和停车状态。
指令矩阵测试会从独立冷启动依次验证前进、侧移、转向及每段停止后的稳定站立。
全速包线脚本一次只测试一个轴，并检查起身后站立、20 秒满量程运动、稳态速度误差、
姿态 RMS/峰值、机身高度、求解器有效性、受控停车和 10 秒停车后站立保持；正负六个
方向应分别冷启动 Gazebo 验收。当前配置已通过 `vx +/-1.5 m/s`、`vy +/-1.0 m/s` 和
`yaw +/-2.0 rad/s` 六项独立冷启动测试，详细数据见仓库根目录 README。

完整依赖安装、架构说明、验收指标和真机测试顺序见仓库根目录 README。
