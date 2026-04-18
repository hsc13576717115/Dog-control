# qr_guide

`qr_guide` 是当前四足机器人 ROS 2 主控包，负责把 IMU 输入、手柄输入、状态估计、状态机控制和真实电机通信串成一条稳定的真机控制链路。

## 1. 包定位

- ROS 2 package：`qr_guide`
- 可执行文件：`junior_ctrl`
- 主 launch：`launch/dog.launch.py`
- 参数文件：`config/custom_quadruped.yaml`

当前包的职责边界：

- 接收外部输入：`/imu`、`/joy`
- 运行控制主循环
- 维护状态机与估计器
- 输出 12 个关节命令
- 与真实电机串口通信
- 发布 RViz 和调试所需的话题

## 2. 代码框架

目录说明：

| 目录 | 核心文件 | 作用 |
| --- | --- | --- |
| `config/` | `custom_quadruped.yaml` | 机器人参数唯一来源 |
| `include/config` / `src/config` | `RobotConfig.*` | 参数结构体、YAML 加载、合法性检查 |
| `include/model` / `src/model` | `LegKinematics.*`, `RobotModel.*` | 单腿 FK / IK / Jacobian 与整机模型 |
| `include/input` / `src/input` | `JoystickMapper.*` | 手柄按键和摇杆映射 |
| `include/runtime` / `src/runtime` | `ControllerNode.*`, `RobotRunner.*`, `VisualizationPublisher.*` | ROS 输入采集、主循环调度、可视化发布 |
| `include/interface` / `src/interface` | `IOSDK.*` | 真机串口通信、减速比换算、校准、状态回填 |
| `include/control` / `src/control` | `Estimator.*`, `HybridStandController.*` | 状态估计和站立力位混合控制 |
| `include/FSM` / `src/FSM` | `FSM.*`, `State_*.*` | 主状态机及各状态实现 |
| `src/main.cpp` | `main.cpp` | 程序入口，完成对象装配与线程调度 |
| `scripts/` | `event2joy.py` | Linux `event*` 到 ROS `/joy` 的适配层 |
| `launch/` | `dog.launch.py` | 一键启动 IMU、手柄、主控、URDF 与 RViz |

## 3. 主控制流程

控制主循环的数据流：

```text
/imu + /joy
    -> ControllerNode
    -> RobotRunner::step()
    -> JoystickMapper
    -> IOSDK::sendRecv()
    -> Estimator::run()
    -> FSM::run()
    -> VisualizationPublisher::publish()
```

更细的执行顺序：

1. `main.cpp`
   读取安装目录中的参数文件，创建 `ControllerNode`、`IOSDK`、`QuadrupedRobot`、`ControllerContext`。
2. `ControllerNode`
   只做一件事：订阅 `/imu` 和 `/joy`，并在主循环里提供一致性的输入快照。
3. `RobotRunner`
   是整个包的周期调度中心，负责把输入、硬件、估计器、状态机、可视化串起来。
4. `JoystickMapper`
   将 `Joy` 消息转换成内部使用的 `UserCommand` 与 `UserValue`。
5. `IOSDK`
   完成电机命令发送、反馈读取、减速比换算、校准偏差处理。
6. `Estimator`
   依据 IMU、关节状态和接触/相位信息估计机体位置与速度。
7. `FSM`
   根据当前状态计算 12 关节的期望位置、速度和前馈力矩。
8. `VisualizationPublisher`
   发布 odom、轨迹、状态文字、足迹和关节角，用于 RViz 和调试。

## 4. 运行时核心对象

### 4.1 `ControllerContext`

`ControllerContext` 是整条控制链共享的上下文对象，主要保存：

- `lowCmd`：当前 12 电机命令
- `lowState`：当前 12 电机状态 + IMU + 用户命令
- `ioInter`：真实硬件或其他 IO 实现
- `robotModel`：运动学和整机模型
- `estimator`：状态估计器
- `parameters`：启动时加载的机器人参数
- `contact / phase`：当前接触状态与步态相位

它的作用是把原本散落在各模块里的共享数据集中管理，降低耦合。

### 4.2 `RobotRunner`

`RobotRunner` 是主循环“总调度器”，它负责：

- 读取 ROS 输入快照
- 同步 IMU 到 `LowlevelState`
- 解析手柄命令
- 调用 `IOSDK` 完成硬件收发
- 处理校准完成后的状态重置
- 调用估计器
- 调用状态机
- 发布 RViz 可视化

如果后续要排查“控制链为什么没有动作”，优先看这里。

## 5. 状态机

当前主线只保留 4 个状态：

| 状态 | 作用 |
| --- | --- |
| `PASSIVE` | 电机保持无力或安全待机 |
| `FIXED_STAND` | 站立插值与站立保持 |
| `TROTTING` | 轻量级小跑控制 |
| `STEP_TEST` | 大步/跳跃等动作验证 |

当前常用切换关系：

| 当前状态 | 触发命令 | 下一状态 |
| --- | --- | --- |
| `PASSIVE` | `L2_A` | `FIXED_STAND` |
| `FIXED_STAND` | `L2_B` | `PASSIVE` |
| `FIXED_STAND` | `L2_X` | `TROTTING` |
| `TROTTING` | `L2_B` | `PASSIVE` |
| `TROTTING` | `L2_A` | `FIXED_STAND` |
| `TROTTING` | `L2_Y` | `STEP_TEST` |
| `STEP_TEST` | 动作完成或再次切换 | 返回 `TROTTING / FIXED_STAND / PASSIVE` |

说明：

- 未校准前，手柄只允许 `START` 触发校准，不允许直接进入动作状态。
- 状态切换命令由 `JoystickMapper` 统一解释，状态内部只关心 `UserCommand`。

## 6. 运动学与坐标系

### 6.1 统一坐标系

内部控制坐标统一为：

- `x` 向前
- `y` 向机器人右侧
- `z` 向上

因此：

- `FR / RR` 的 `y` 必须为正
- `FL / RL` 的 `y` 必须为负

### 6.2 运动学接口

核心实现：

- `LegKinematics`
  负责单腿 `FK / IK / Jacobian`
- `RobotModel`
  负责把四条腿封装成整机接口
- `QuadrupedRobot`
  负责兼容旧接口并转发到新的参数化模型

当前统一使用你已有的关节定义和镜像约定，不再把 FK / IK 逻辑散落在各个状态文件里。

## 7. ROS 接口

### 7.1 输入 Topic

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/imu` | `sensor_msgs/msg/Imu` | 机身姿态、角速度、线加速度 |
| `/joy` | `sensor_msgs/msg/Joy` | 状态切换按钮和摇杆输入 |

### 7.2 输出 Topic

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/qr_guide/estimation/odom` | `nav_msgs/msg/Odometry` | 机体状态估计 |
| `/qr_guide/estimation/path` | `nav_msgs/msg/Path` | 机身轨迹 |
| `/qr_guide/visualization/marker_array` | `visualization_msgs/msg/MarkerArray` | 调试可视化 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 实际关节状态 |
| `/tf` | TF | `odom -> base_link_est` 等变换 |

### 7.3 launch 参数

`launch/dog.launch.py` 常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `imu_serial_port` | `/dev/ttyUSB0` | IMU 串口 |
| `imu_serial_baud` | `921600` | IMU 波特率 |
| `joy_event_path` | `auto` | 手柄事件设备；默认自动优先选择名字匹配 Xbox 的 `/dev/input/event*` |
| `use_rviz` | `true` | 是否启动 RViz |

## 8. 参数文件详解

参数文件：

- `config/custom_quadruped.yaml`

主要参数块及含义：

### 8.1 `robot.mass`

重要参数：

- `total_mass_kg`：整机总质量
- `body_mass_kg`：机身质量
- `leg_mass_each_kg / thigh_mass_each_kg / calf_mass_each_kg`：腿部分质量
- `com_offset_m`：质心相对机体系原点偏移
- `whole_robot_inertia_kg_m2_diag`：整机对角惯量

影响：

- 状态估计器和控制器中的质量、惯量建模
- RViz 和文档中的机身尺度理解

### 8.2 `robot.hip_mounts_in_body`

四条腿髋关节安装点，定义在 `body frame` 下。

影响：

- 整机运动学
- 足端目标的 body/hip 坐标转换
- URDF 动态生成结果

### 8.3 `robot.leg_geometry`

重要参数：

- `l0`：髋轴到腿平面的横向偏移
- `l1`：大腿长度
- `l2`：小腿长度
- `foot_radius_m`：脚端半径

影响：

- FK / IK / Jacobian 结果
- 状态估计器 `_feetH` 的参考高度
- RViz 腿部模型尺寸

### 8.4 `robot.drive`

重要参数：

- `serial_ports`：`FR / FL / RR / RL` 串口映射
- `use_parallel_leg_io`：是否并发进行四腿串口收发
- `base_gear_ratio`：髋与大腿减速比
- `calf_total_gear_ratio`：小腿总减速比

影响：

- 真机串口连接是否正确
- 电机角度命令与反馈的换算
- 系统通信稳定性

### 8.5 `robot.joint_limits`

定义三个关节的机械限位。

影响：

- IK 输出会在状态内再次做限幅保护
- 防止目标角越界导致危险动作

### 8.6 `robot.stand_targets`

重要参数：

- `normal_feet_in_hip`
- `crouch_feet_in_hip`

影响：

- `FIXED_STAND` 和部分动作状态的目标足端位置
- 站姿高度与腿部展开程度

### 8.7 `robot.hybrid_stand`

`FIXED_STAND` 使用的力位混合参数。

重点参数：

- `kp_z / kd_z`
- `kp_roll / kd_roll`
- `kp_pitch / kd_pitch`
- `fz_min_per_leg_n / fz_max_per_leg_n`
- `tau_limit_nm`
- `tau_rate_limit_nm_per_s`

说明：

- 当前默认 `enabled: false`
- 如果后续开启，需要和 IMU 坐标、实机增益一起联调

## 9. 校准逻辑

校准由 `IOSDK` 执行，当前逻辑：

1. `IOSDK` 启动后先发一段很小的预对位力矩：
   - 髋关节统一向内展
   - 大腿先向下压
   - 小腿随后向上抬
2. 预对位结束后，手动确认机器人已经落到约定的 L 型姿态。
3. 按下 `START`。
4. 读取当前电机回传角度。
5. 用“当前角度 - 理想零位角”计算 `_calibOffset`。
6. 后续：
   - 下发命令：`q_motor = q_user + calib_offset`
   - 读取反馈：`q_user = q_feedback - calib_offset`

当前校准目标角：

- 髋关节：`0.0 deg`
- 大腿：`-161.8 deg`
- 小腿：`-71.8 deg`

## 10. 可视化链路

`VisualizationPublisher` 负责发布：

- `odom`
- `path`
- `joint_states`
- `MarkerArray`
- `TF`

显示内容包括：

- 机身估计位姿
- 机身轨迹
- 四条腿实际足端轨迹
- 命令足端/状态信息
- 速度箭头和状态文本

同时，`launch/dog.launch.py` 会基于参数文件动态生成 URDF，可确保 RViz 里的机身和腿长与参数一致。

## 11. 构建与运行

编译：

```bash
cd /home/orangepi/qr_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select qr_guide
```

运行：

```bash
cd /home/orangepi/qr_ws
source install/setup.bash
ros2 launch qr_guide dog.launch.py
```

关闭 RViz：

```bash
ros2 launch qr_guide dog.launch.py use_rviz:=false
```

## 12. 调试建议

出现问题时建议按这个顺序排查：

1. `ros2 topic echo /imu` 是否有稳定输出。
2. `ros2 topic echo /joy` 是否能看到按钮和摇杆变化。
3. 检查 `custom_quadruped.yaml` 中的串口映射是否正确。
4. 先只做校准，不切状态。
5. 再验证 `PASSIVE -> FIXED_STAND`。
6. 再验证 `FIXED_STAND -> TROTTING`。
7. 最后验证 `STEP_TEST`。

推荐优先阅读的代码文件：

- `src/main.cpp`
- `src/runtime/RobotRunner.cpp`
- `src/interface/IOSDK.cpp`
- `src/control/Estimator.cpp`
- `src/FSM/FSM.cpp`
- `src/FSM/State_FixedStand.cpp`
- `src/FSM/State_Trotting.cpp`
