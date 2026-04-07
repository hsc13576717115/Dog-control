# qr_ws

`qr_ws` 是当前四足机器人的 ROS2 Humble 工作空间。现在工作空间已经完成一轮清理和重构，主控链路只保留当前真实在用的包和代码，不再混着旧 ROS1 风格控制框架和大量遗留模块。

## 1. 工作空间位置

默认工作空间路径：

```bash
/home/orangepi/qr_ws
```

主要目录结构：

```text
qr_ws/
├── src/
│   ├── fdilink_ahrs/
│   ├── serial_ros2/
│   └── qr_guide/
├── build/
├── install/
└── log/
```

说明：

- `src/` 是源码
- `build/ install/ log/` 是 `colcon build` 生成的编译产物

## 2. 三个主要包

### 2.1 `fdilink_ahrs`

负责读取 FDILink IMU 数据，并发布：

- `/imu`

这是主控制器的姿态输入来源。

### 2.2 `serial_ros2`

提供串口库支持。

说明：

- 目录名是 `serial_ros2`
- 导出的 ROS2 包名是 `serial`
- `fdilink_ahrs` 会依赖它

### 2.3 `qr_guide`

这是当前主控制包，负责：

- 订阅 `/imu`
- 订阅 `/joy`
- 运行状态机和控制器
- 通过串口和四条腿电机通信

包内更详细的说明见：

- [src/qr_guide/README.md](src/qr_guide/README.md)

## 3. 当前主控链路

整个系统现在可以理解成这条链路：

```text
FDILink IMU
  -> fdilink_ahrs
  -> /imu
  -> qr_guide(junior_ctrl)
  -> runtime + FSM + estimator
  -> IOSDK
  -> /dev/ttyS3 / ttyS4 / ttyS7 / ttyS8
  -> 四条腿电机

手柄
  -> /dev/input/event*
  -> event2joy.py
  -> /joy
  -> qr_guide(junior_ctrl)
```

校准完成后，`junior_ctrl` 现在还会输出两类动态调试信息：

- 关节角和足端 FK 打印
- 状态估计的 `position / velocity / contact / phase`

同时还会发布一套 RViz 可视化 topic：

- `/qr_guide/estimation/odom`
- `/qr_guide/estimation/path`
- `/qr_guide/visualization/marker_array`
- `/joint_states`
- `/tf`
- `/robot_description`

## 4. `qr_guide` 当前架构

`qr_guide` 现在主线只保留这些模块：

- `config/`
  - 机器人参数唯一来源
- `model/`
  - 单腿运动学和整机模型
- `input/`
  - 手柄映射
- `runtime/`
  - ROS2 输入采集和控制循环调度
- `interface/`
  - 真实电机串口通信、校准和回读
- `control/`
  - 状态估计器
- `FSM/`
  - 主状态机

当前主状态只有 4 个：

- `PASSIVE`
- `FIXED_STAND`
- `TROTTING`
- `STEP_TEST`

旧的 `FreeStand / BalanceTest / SwingTest / move_base` 以及配套旧控制器、旧步态模块已经从仓库中删除。

## 5. 坐标系与参数

当前主线统一采用这套坐标系：

- `body / hip frame`: `x` 向前，`y` 向机器人右侧，`z` 向上
- `FR / RR` 的 `y` 为正
- `FL / RL` 的 `y` 为负

说明：

- 这套是控制器内部的工作坐标系
- RViz 显示层会单独对整狗几何、轨迹和估计位置做一次左右反射，把显示转换成更符合 RViz 使用习惯的显示坐标
- IMU 姿态本身保持原始 ROS 方向，不做偏航镜像
- 所以控制器和运动学不需要改，RViz 里的左右腿也不会再对调

机器人主参数统一放在：

- [src/qr_guide/config/custom_quadruped.yaml](src/qr_guide/config/custom_quadruped.yaml)

主程序启动时会从安装目录读取这份参数，而不是从源码目录硬编码读取。

当前状态估计器里的 `_feetH` 也来自这份参数：

- `robot.leg_geometry.foot_radius_m = 0.025`

这对应“世界系原点在地面、足端观测点位于脚端半径中心”的假设。

## 6. 构建

在工作空间根目录执行：

```bash
cd /home/orangepi/qr_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select qr_guide fdilink_ahrs
```

如果只想单独编译主控包：

```bash
colcon build --packages-select qr_guide
```

## 7. 运行

完整启动：

```bash
cd /home/orangepi/qr_ws
source install/setup.bash
ros2 launch qr_guide dog.launch.py
```

这会启动：

1. `fdilink_ahrs/ahrs_driver_node`
2. `qr_guide/event2joy.py`
3. `qr_guide/junior_ctrl`
4. `robot_state_publisher`
5. `rviz2`（默认开启）

当前 RViz 里默认会显示：

- 状态估计的 `odom/path`
- `odom -> base_link_est` TF
- 基于实时 `/joint_states` 的完整狗模型
- 四个实际足端和命令足端轨迹
- 速度箭头、状态文字和接触/相位信息

当前整狗模型已经往宇树 Go2 的工业设计语言做了一版美化：机身更圆润，顶部增加传感器造型，腿部视觉件改成细圆柱加球帽的胶囊风格，关节也换成更圆润的球形显示，和轨迹叠加时会比早期的方块连杆更清爽。

如果只想运行控制，不开 RViz：

```bash
ros2 launch qr_guide dog.launch.py use_rviz:=false
```

## 8. 真机运行前检查

建议按下面顺序检查：

1. `/imu` 是否正常
2. `/joy` 是否正常
3. 4 个串口是否和 `FR / FL / RR / RL` 一一对应
4. 先在离地状态完成校准
5. 先验证 `PASSIVE -> FIXED_STAND`
6. 再验证 `FIXED_STAND -> TROTTING`
7. 最后验证 `STEP_TEST`

## 9. 文档约定

以后只要主线代码结构、参数来源、坐标系或控制流程发生变化，都应该同时更新两份 README：

- 工作空间总览：
  - [README.md](/home/orangepi/qr_ws/README.md)
- `qr_guide` 包内详细说明：
  - [src/qr_guide/README.md](/home/orangepi/qr_ws/src/qr_guide/README.md)
