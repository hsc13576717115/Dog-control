# Dog-control ROS2 Humble 工作空间说明

这个工程是基于 Ubuntu 22.04 + ROS2 Humble 的四足机器人控制工作空间，工作空间名字是 `qr_ws`。

原来的 ROS1 `unitree_guide` 已经迁移成 ROS2 的 `qr_guide`。迁移时尽量保持原来的控制逻辑、状态机和步态框架不变，主要改的是 ROS 通信层、构建方式和启动方式。

---

## 1. 先说清楚：这个工程是干什么的

这个工作空间主要做 3 件事：

1. 读取 IMU 数据
2. 读取手柄输入
3. 把 IMU 和手柄输入送进四足控制器，再通过串口发给电机

你可以把整个工程理解成下面这条链路：

```text
FDILink IMU
  -> fdilink_ahrs
  -> /imu
  -> qr_guide(junior_ctrl)
  -> FSM状态机 + 控制器
  -> IOSDK串口下发
  -> /dev/ttyS3 / ttyS4 / ttyS7 / ttyS8
  -> 四条腿的电机

手柄
  -> /dev/input/event6
  -> event2joy.py
  -> /joy
  -> qr_guide(junior_ctrl)
```

---

## 2. 工作空间位置

建议工作空间固定放在：

```bash
/home/orangepi/qr_ws
```

当前主要目录：

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

---

## 3. 三个包分别负责什么

### 3.1 `fdilink_ahrs`

作用：读取 FDILink IMU 串口数据，并发布 ROS2 话题。

你最关心的是：

- 发布 `/imu`
- 这是主控制器姿态输入的来源

### 3.2 `serial_ros2`

作用：提供 `serial` 库。

说明：

- 这个目录名叫 `serial_ros2`
- 但它导出的 ROS2 包名是 `serial`
- `fdilink_ahrs` 会用到它

### 3.3 `qr_guide`

作用：主控制包。

它负责：

- 接收 `/imu`
- 接收 `/joy`
- 运行 FSM 状态机
- 调用控制器
- 通过串口和电机通信

---

## 4. 小白先看哪几个文件

如果你第一次接触这个工程，建议按下面顺序看：

1. [dog.launch.py](src/qr_guide/launch/dog.launch.py)
2. [main.cpp](src/qr_guide/src/main.cpp)
3. [ControlFrame.cpp](src/qr_guide/src/control/ControlFrame.cpp)
4. [FSM.cpp](src/qr_guide/src/FSM/FSM.cpp)
5. [CtrlComponents.h](src/qr_guide/include/control/CtrlComponents.h)
6. [IOSDK.cpp](src/qr_guide/src/interface/IOSDK.cpp)
7. [event2joy.py](src/qr_guide/scripts/event2joy.py)

这样看最容易建立整体理解。

---

## 5. 整个控制框架怎么跑

### 5.1 启动入口

整个系统从这个 launch 启动：

[dog.launch.py](src/qr_guide/launch/dog.launch.py)

它会同时启动 3 个节点：

1. `fdilink_ahrs/ahrs_driver_node`
2. `qr_guide/event2joy.py`
3. `qr_guide/junior_ctrl`

### 5.2 主控制入口

主控制入口在：

[main.cpp](src/qr_guide/src/main.cpp)

它的职责是：

1. 初始化 ROS2
2. 订阅 `/imu`
3. 创建 `IOSDK`
4. 创建 `CtrlComponents`
5. 创建 `ControlFrame`
6. 在循环里把 IMU 数据写入 `lowState->imu`
7. 调用 `ctrlFrame.run()`

### 5.3 控制主循环

[ControlFrame.cpp](src/qr_guide/src/control/ControlFrame.cpp) 很简单，它内部主要调用：

```text
FSM::run()
```

而 [FSM.cpp](src/qr_guide/src/FSM/FSM.cpp) 是整个控制调度中心。

`FSM::run()` 里面大致顺序是：

1. `sendRecv()` 和底层硬件通信
2. `runWaveGen()` 计算步态相位
3. `estimator->run()` 做状态估计
4. 当前状态 `run()`
5. 判断是否切换状态

### 5.4 状态机层

状态机在：

- [include/FSM](src/qr_guide/include/FSM)
- [src/FSM](src/qr_guide/src/FSM)

当前主要状态有：

- `PASSIVE`：被动态
- `FIXEDSTAND`：固定站立
- `FREESTAND`：自由站立
- `TROTTING`：小跑步态
- `BALANCETEST`：平衡测试
- `SWINGTEST`：摆腿测试
- `STEPTEST`：步态/跳跃测试

你如果想改“机器人做什么动作”，一般就是从这里入手。

### 5.5 控制组件层

[CtrlComponents.h](src/qr_guide/include/control/CtrlComponents.h) 是控制共享数据的总入口。

里面放了这些关键对象：

- `lowCmd`
- `lowState`
- `ioInter`
- `robotModel`
- `waveGen`
- `estimator`
- `balCtrl`

你可以把它理解成“整个控制系统的公共上下文”。

### 5.6 硬件接口层

硬件接口在：

- [include/interface](src/qr_guide/include/interface)
- [IOSDK.cpp](src/qr_guide/src/interface/IOSDK.cpp)

这里负责：

1. 订阅 `/joy`
2. 把手柄输入转换成 `UserCommand` 和 `UserValue`
3. 和 4 路串口电机通信
4. 把控制命令发给电机
5. 把电机反馈写回 `lowState`

这部分是“控制算法”和“真实硬件”之间的桥。

---

## 6. 代码框架怎么理解

`qr_guide` 里面的代码可以按职责分成下面几层：

### 6.1 `include/common` 和 `src/common`

作用：

- 数学工具
- 时间工具
- 机器人基础模型
- 滤波器

这是底层工具层。

### 6.2 `include/message`

作用：

- 定义控制命令
- 定义底层状态
- 定义内部消息结构

这是系统内部数据结构层。

### 6.3 `include/Gait` 和 `src/Gait`

作用：

- 生成步态相位
- 计算足端轨迹

这是步态生成层。

### 6.4 `include/control` 和 `src/control`

作用：

- 控制框架
- 状态估计
- 平衡控制

这是算法调度层。

### 6.5 `include/FSM` 和 `src/FSM`

作用：

- 管理机器人状态
- 决定当前执行哪个动作

这是行为决策层。

### 6.6 `include/interface` 和 `src/interface`

作用：

- 对接 ROS2
- 对接手柄
- 对接电机串口

这是硬件交互层。

---

## 7. 硬件框架怎么理解

你当前这套系统可以按下面理解：

```text
Orange Pi 5 Plus
├── IMU
│   └── /dev/ttyUSB0
├── 手柄
│   └── /dev/input/event6
├── 4路电机串口
│   ├── /dev/ttyS3
│   ├── /dev/ttyS4
│   ├── /dev/ttyS7
│   └── /dev/ttyS8
└── Unitree 电机 SDK
    └── /home/orangepi/unitree_actuator_sdk-main/unitree_actuator_sdk-main
```

### 7.1 IMU

IMU 通过 `fdilink_ahrs` 进入 ROS2，输出 `/imu`。

### 7.2 手柄

手柄通过 [event2joy.py](src/qr_guide/scripts/event2joy.py) 读取 `/dev/input/event6`，再发布 `/joy`。

### 7.3 电机

电机串口在 [IOSDK.cpp](src/qr_guide/src/interface/IOSDK.cpp) 里固定写成了：

- `/dev/ttyS3`
- `/dev/ttyS4`
- `/dev/ttyS7`
- `/dev/ttyS8`

如果以后串口号变了，就改这个文件。

---

## 8. 如何启动

### 8.1 第一次使用先编译

```bash
source /opt/ros/humble/setup.bash
cd /home/orangepi/qr_ws
colcon build --symlink-install
source install/setup.bash
```

### 8.2 如有需要，先配置 IMU 权限

```bash
sudo bash /home/orangepi/qr_ws/src/fdilink_ahrs/wheeltec_udev.sh
```

### 8.3 正常启动整套系统

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/qr_ws/install/setup.bash
ros2 launch qr_guide dog.launch.py
```

### 8.4 常用启动参数

```bash
ros2 launch qr_guide dog.launch.py \
  imu_serial_port:=/dev/ttyUSB0 \
  imu_serial_baud:=921600 \
  joy_event_path:=/dev/input/event6
```

---

## 9. 启动前建议检查

启动前建议先看一下设备在不在：

```bash
ls /dev/ttyUSB0
ls /dev/input/event6
ls /dev/ttyS3 /dev/ttyS4 /dev/ttyS7 /dev/ttyS8
```

如果某个设备不存在，程序大概率起不来，或者起了也没法正常工作。

---

## 10. 启动后怎么判断系统是不是正常

### 10.1 看 IMU 话题

```bash
ros2 topic echo /imu
```

如果 `/imu` 有数据，说明 `fdilink_ahrs` 正常。

### 10.2 看手柄话题

```bash
ros2 topic echo /joy
```

如果按动手柄时 `/joy` 有变化，说明 `event2joy.py` 正常。

### 10.3 看控制节点是否在运行

```bash
ros2 node list
```

正常情况下你应该能看到类似这些节点：

- `/ahrs_driver`
- `/event2joy_node`
- `/qr_junior_ctrl`

---

## 11. 如果你以后要改代码，通常改哪里

### 改启动方式

改：

[dog.launch.py](src/qr_guide/launch/dog.launch.py)

### 改手柄设备

优先通过 launch 参数改：

```bash
joy_event_path:=/dev/input/eventX
```

### 改 IMU 串口

优先通过 launch 参数改：

```bash
imu_serial_port:=/dev/xxx
```

### 改动作逻辑

改：

[src/FSM](src/qr_guide/src/FSM)

### 改控制框架

改：

- [FSM.cpp](src/qr_guide/src/FSM/FSM.cpp)
- [ControlFrame.cpp](src/qr_guide/src/control/ControlFrame.cpp)
- [CtrlComponents.h](src/qr_guide/include/control/CtrlComponents.h)

### 改硬件通信

改：

[IOSDK.cpp](src/qr_guide/src/interface/IOSDK.cpp)

---

## 12. 一句话总结这个工程

如果你只记一句话，那就是：

```text
fdilink_ahrs 提供 IMU
event2joy.py 提供手柄输入
qr_guide 负责状态机 + 控制器 + 电机串口通信
```

只要你把这条链路记住，这个工程基本就不会看迷路。
