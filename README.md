# qr_ws

`qr_ws` 是这台四足机器人的 ROS 2 Humble 主工作区。当前仓库已经整理成一条可维护的真机控制主线，重点围绕 IMU 驱动、手柄输入、状态估计、状态机控制和电机串口通信展开。

## 1. 工作区定位

- 工作区路径：`/home/orangepi/qr_ws`
- 目标平台：`ROS 2 Humble`
- 主要用途：真机四足控制、姿态输入、手柄输入、RViz 可视化、串口电机通信

目录结构：

```text
qr_ws/
├── src/
│   ├── fdilink_ahrs/     # FDILink IMU 串口驱动与话题发布
│   ├── serial_ros2/      # 串口基础库，供 fdilink_ahrs 等包复用
│   └── qr_guide/         # 四足主控包：状态机、估计器、模型、真机 IO
├── build/
├── install/
└── log/
```

说明：

- `src/` 保存源码。
- `build/ install/ log/` 为 `colcon build` 生成目录。
- 当前推荐直接在这个工作区内完成开发、编译和真机调试。

## 2. 整体系统框架

工作区主链路可以理解为两条输入链，最终汇入 `qr_guide` 控制器：

```text
FDILink IMU
  -> fdilink_ahrs
  -> /imu
  -> qr_guide::ControllerNode
  -> RobotRunner
  -> Estimator + FSM
  -> IOSDK
  -> /dev/ttyS3 / ttyS4 / ttyS7 / ttyS8
  -> 四条腿电机

手柄 /dev/input/event*
  -> qr_guide/scripts/event2joy.py
  -> /joy
  -> qr_guide::ControllerNode
  -> JoystickMapper
  -> FSM 状态切换 / 速度命令
```

控制和可视化的输出链路：

```text
qr_guide
  -> /joint_states
  -> /tf
  -> /robot_description
  -> /qr_guide/estimation/odom
  -> /qr_guide/estimation/path
  -> /qr_guide/visualization/marker_array
  -> RViz
```

## 3. 三个 package 的职责

### 3.1 `fdilink_ahrs`

作用：

- 通过串口读取 FDILink IMU / AHRS / GPS 数据帧
- 做 CRC 校验和序号检查
- 将姿态、角速度、线加速度转换为 ROS 2 消息
- 发布 `/imu` 等调试与导航相关话题

更多说明见：

- [src/fdilink_ahrs/README.md](src/fdilink_ahrs/README.md)

### 3.2 `serial_ros2`

作用：

- 提供跨平台串口基础库
- 作为 `fdilink_ahrs` 的底层依赖
- 当前在本工作区里主要被当作第三方基础组件使用

更多说明见：

- [src/serial_ros2/README.md](src/serial_ros2/README.md)

### 3.3 `qr_guide`

作用：

- 订阅 `/imu` 和 `/joy`
- 运行状态机和状态估计
- 根据当前状态生成 12 个关节目标
- 通过 `IOSDK` 与四条腿电机串口通信
- 发布 RViz 所需的状态估计、轨迹、TF、关节角和可视化 Marker

更多说明见：

- [src/qr_guide/README.md](src/qr_guide/README.md)

## 4. `qr_guide` 的主控框架

当前主线按模块划分如下：

| 模块 | 目录 | 作用 |
| --- | --- | --- |
| 参数配置 | `config/` | 统一加载机器人尺寸、质量、串口、站姿、速度限制等参数 |
| 模型层 | `model/` | 单腿 FK / IK / Jacobian 与整机模型接口 |
| 输入层 | `input/` | 手柄到 `UserCommand / UserValue` 的映射 |
| 运行时 | `runtime/` | ROS 输入采集、主循环调度、RViz 可视化发布 |
| 硬件接口 | `interface/` | 电机串口通信、减速比换算、校准偏差、状态回填 |
| 控制层 | `control/` | 状态估计与站立力位混合控制 |
| 状态机 | `FSM/` | `PASSIVE / FIXED_STAND / TROTTING / STEP_TEST` 四个主状态 |

主程序执行顺序：

1. `main.cpp` 从安装目录加载参数文件。
2. `ControllerNode` 订阅 `/imu` 和 `/joy`。
3. `RobotRunner` 以固定周期读取输入快照。
4. `JoystickMapper` 解析手柄命令。
5. `IOSDK` 收发电机命令和状态，并处理校准偏差。
6. `Estimator` 用 IMU、关节状态、接触相位估计机身状态。
7. `FSM` 根据当前状态生成 12 关节命令。
8. `VisualizationPublisher` 发布 odom、path、joint_states、TF 和 Marker。

## 5. 对外接口总览

### 5.1 输入接口

| 类型 | 名称 | 来源 | 用途 |
| --- | --- | --- | --- |
| Topic | `/imu` | `fdilink_ahrs` | 提供机身姿态、角速度、线加速度 |
| Topic | `/joy` | `event2joy.py` | 提供状态切换按钮和摇杆速度输入 |
| 串口 | `/dev/ttyS3/ttyS4/ttyS7/ttyS8` | `qr_guide::IOSDK` | 4 条腿电机通信 |
| 串口 | `/dev/ttyUSB0` | `fdilink_ahrs` | IMU 通信 |

### 5.2 主要输出接口

| 类型 | 名称 | 发布包 | 用途 |
| --- | --- | --- | --- |
| Topic | `/imu` | `fdilink_ahrs` | 供控制器订阅 |
| Topic | `/qr_guide/estimation/odom` | `qr_guide` | 状态估计 odom |
| Topic | `/qr_guide/estimation/path` | `qr_guide` | 机身轨迹 |
| Topic | `/qr_guide/visualization/marker_array` | `qr_guide` | 足迹、机身和状态文字 |
| Topic | `/joint_states` | `qr_guide` | RViz / `robot_state_publisher` 使用 |
| TF | `odom -> base_link_est` | `qr_guide` | RViz 位姿显示 |
| 参数 | `robot_description` | `qr_guide` launch | 运行期生成 URDF 模型 |

## 6. 坐标系与关键约定

控制主线统一使用：

- `body / hip frame`：`x` 向前，`y` 向机器人右侧，`z` 向上
- 腿序固定：`FR / FL / RR / RL`
- 因此 `FR / RR` 的 `y` 为正，`FL / RL` 的 `y` 为负

补充说明：

- 控制器、运动学、校准、估计器内部都按这套坐标系工作。
- RViz 为了显示直观，会对几何做一次左右反射，把显示坐标转换成更符合 ROS 使用习惯的形式。
- IMU 姿态仍保持 ROS 消息方向，不在可视化层额外镜像偏航。

## 7. 关键参数入口

机器人主参数统一位于：

- `src/qr_guide/config/custom_quadruped.yaml`

最重要的参数分组：

| 参数组 | 作用 |
| --- | --- |
| `robot.body_size_m` | 机身体积和可视化尺寸 |
| `robot.mass.*` | 质量、惯量、质心偏移 |
| `robot.hip_mounts_in_body` | 四条腿髋关节安装点 |
| `robot.leg_geometry` | `l0/l1/l2` 和脚端半径 |
| `robot.drive.serial_ports` | 四条腿电机串口映射 |
| `robot.drive.base_gear_ratio` | 髋/大腿减速比 |
| `robot.drive.calf_total_gear_ratio` | 小腿总减速比 |
| `robot.joint_limits` | 三个关节机械限位 |
| `robot.stand_targets.*` | 站立与匍匐目标足端位置 |
| `robot.hybrid_stand.*` | FixedStand 力位混合控制参数 |
| `robot.velocity_limits.*` | 小跑等状态的速度限幅 |

推荐优先关注的参数：

- `hip_mounts_in_body`
- `leg_geometry.l0/l1/l2`
- `drive.serial_ports`
- `joint_limits`
- `stand_targets.normal_feet_in_hip`
- `leg_geometry.foot_radius_m`

## 8. 构建

在工作区根目录执行：

```bash
cd /home/orangepi/qr_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select serial fdilink_ahrs qr_guide
```

如果只重编主控包：

```bash
colcon build --packages-select qr_guide
```

编译完成后：

```bash
source /home/orangepi/qr_ws/install/setup.bash
```

## 9. 运行

完整启动：

```bash
cd /home/orangepi/qr_ws
source install/setup.bash
ros2 launch qr_guide dog.launch.py
```

常用 launch 参数：

```bash
ros2 launch qr_guide dog.launch.py \
  imu_serial_port:=/dev/ttyUSB0 \
  imu_serial_baud:=921600 \
  joy_event_path:=/dev/input/event6 \
  use_rviz:=true
```

这条 launch 会启动：

1. `fdilink_ahrs/ahrs_driver_node`
2. `qr_guide/event2joy.py`
3. `qr_guide/junior_ctrl`
4. `robot_state_publisher`
5. `rviz2`

## 10. 真机上电前检查顺序

建议按下面顺序确认：

1. `/imu` 是否稳定输出。
2. `/joy` 是否正常变化。
3. `FR / FL / RR / RL` 串口映射是否正确。
4. 先在离地状态完成校准。
5. 先测试 `PASSIVE -> FIXED_STAND`。
6. 再测试 `FIXED_STAND -> TROTTING`。
7. 最后测试 `STEP_TEST`。

## 11. 推荐的 README 组织形式

当前工作区文档采用“总览 README + package README”结构，这是比较适合长期维护的形式：

- 工作区根 README：回答“这个仓库整体做什么、模块怎么连接、如何构建运行”。
- package README：回答“这个包内部怎么工作、有哪些接口、关键参数是什么、调试时看哪里”。
- 参数文件注释：回答“每个参数为什么存在、改了会影响什么”。
- 关键代码文件注释：回答“流程从哪里进、关键数据怎么流动、哪些约定不能改”。

这种形式适合你后续继续维护，因为它把“总览、模块说明、细节参数”分开了，不会把所有信息都堆到一个文件里。
