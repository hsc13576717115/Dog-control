# 机器狗控制系统

[![ROS 版本](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![平台](https://img.shields.io/badge/platform-Orange%20Pi-orange.svg)](https://www.orangepi.org/)
[![许可证](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

**Language** / 语言: [English](README.md) | [中文](README.zh-CN.md)

基于 Unitree 机器人框架的四足机器人控制系统，实现了高级步态控制和状态机管理，用于机器狗应用。

## 目录

- [项目概述](#项目概述)
- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [硬件要求](#硬件要求)
- [软件依赖](#软件依赖)
- [安装指南](#安装指南)
- [使用方法](#使用方法)
- [项目结构](#项目结构)
- [配置说明](#配置说明)
- [FSM 状态](#fsm-状态)
- [贡献指南](#贡献指南)
- [许可证](#许可证)

## 项目概述

本项目为四足机器人提供全面的控制系统，专为 Unitree GOM-8010-6电机平台设计。系统采用有限状态机（FSM）架构，集成多种步态和控制模式，结合 IMU 传感器融合实现精确运动控制。

### 核心组件

- **unitree_guide**: 主控制包，实现步态生成、状态管理和机器人控制
- **fdilink_ahrs**: IMU/AHRS 驱动包，用于传感器数据采集

## 功能特性

### 运动控制
- 多种步态模式：小跑（Trotting）、摆动测试、跨步测试
- 平衡测试模式
- 固定站立和自由站立模式
- 底盘移动控制
- 被动模式（安全操作）

### 传感器集成
- FDILINK AHRS IMU 传感器驱动
- 实时姿态估计
- GPS 数据融合
- 磁场感应
- 速度和里程计跟踪

### 控制架构
- 有限状态机（FSM）设计
- 模块化步态生成器
- 足端轨迹规划
- 关节级控制接口

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                       机器狗控制系统                          │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │    IMU      │    │   控制器     │    │   步态       │     │
│  │   驱动      │───▶│    节点     │───▶│  生成器     │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│         │                  │                  │             │
│         ▼                  ▼                  ▼             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │ 传感器      │    │    FSM      │    │  足端       │     │
│  │ 话题        │    │  管理器     │    │  轨迹       │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## 硬件要求

### 最低配置
- **平台**: Orange Pi (RK3588) 或类似的 ARM64 平台
- **内存**: 4GB+ 推荐
- **存储**: 10GB+ 可用空间

### 外设
- FDILINK AHRS IMU 传感器
- Unitree 四足机器人（A1/Go1 系列）
- USB 串口连接（用于 IMU）
- 可选：GPS 模块

## 软件依赖

### 核心依赖
- **ROS**: Noetic Ninjemmys
- **Catkin**: 构建系统
- **C++ 标准**: C++11 或更高

### ROS 包
```bash
roscpp
std_msgs
sensor_msgs
geometry_msgs
nav_msgs
tf
controller_manager
joint_state_controller
robot_state_publisher
unitree_legged_msgs
```

### 系统包
```bash
# Ubuntu/Debian
sudo apt install -y \
    cmake \
    python3-catkin-tools \
    libeigen3-dev
```

## 安装指南

### 1. 克隆仓库

```bash
cd ~/catkin_ws/src
git clone git@github.com:hsc13576717115/Dog-control.git
```

### 2. 安装 ROS 依赖

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译工作空间

```bash
cd ~/catkin_ws
catkin_make
# 或使用 catkin tools
catkin build
```

### 4. 配置工作空间

```bash
source ~/catkin_ws/devel/setup.bash
# 添加到 ~/.bashrc 实现自动加载
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 5. 配置 USB 权限（用于 IMU）

```bash
# 为 FDILINK AHRS 创建 udev 规则
sudo bash ~/catkin_ws/src/fdilink_ahrs/wheeltec_udev.sh
# 或手动将用户添加到 dialout 组
sudo usermod -aG dialout $USER
```

## 使用方法

### 启动控制系统

```bash
roslaunch unitree_guide Dog.launch
```

这将启动：
1. AHRS/IMU 驱动节点
2. 事件到手柄转换节点
3. 主机器人控制节点

### 可用的 ROS 话题

#### 发布的话题
| 话题 | 类型 | 描述 |
|-------|------|-------------|
| `/imu` | `sensor_msgs/Imu` | IMU 传感器数据 |
| `/mag_pose_2d` | `geometry_msgs/Pose2D` | 磁航向 |
| `/euler_angles` | `geometry_msgs/Vector3` | 欧拉角（横滚、俯仰、偏航） |
| `/magnetic` | `geometry_msgs/Vector3` | 磁场强度 |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS 位置数据 |
| `/system_speed` | `geometry_msgs/Twist` | 机体系速度 |
| `/NED_odometry` | `nav_msgs/Odometry` | NED 系里程计 |

### 手动控制

系统支持通过 event 接口进行手柄控制。请确保您的输入设备正确配置。

## 项目结构

```
catkin_ws/
├── src/
│   ├── unitree_guide/           # 主控制包
│   │   ├── include/
│   │   │   ├── FSM/             # 有限状态机状态
│   │   │   │   ├── FSM.h
│   │   │   │   ├── FSMState.h
│   │   │   │   ├── State_Passive.h
│   │   │   │   ├── State_FixedStand.h
│   │   │   │   ├── State_FreeStand.h
│   │   │   │   ├── State_Trotting.h
│   │   │   │   ├── State_SwingTest.h
│   │   │   │   ├── State_StepTest.h
│   │   │   │   ├── State_BalanceTest.h
│   │   │   │   └── State_move_base.h
│   │   │   ├── Gait/            # 步态生成
│   │   │   │   ├── GaitGenerator.h
│   │   │   │   ├── WaveGenerator.h
│   │   │   │   └── FeetEndCal.h
│   │   │   ├── control/         # 控制算法
│   │   │   ├── interface/       # 机器人接口
│   │   │   ├── common/          # 公共工具
│   │   │   ├── message/         # 消息定义
│   │   │   └── thirdParty/      # 第三方库
│   │   ├── src/                 # 源代码
│   │   ├── launch/              # 启动文件
│   │   │   └── Dog.launch
│   │   ├── scripts/             # Python 脚本
│   │   │   └── event2joy.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── fdilink_ahrs/            # IMU 驱动包
│       ├── include/
│       │   ├── ahrs_driver.h
│       │   ├── fdilink_data_struct.h
│       │   └── crc_table.h
│       ├── src/
│       │   ├── ahrs_driver.cpp
│       │   ├── crc_table.cpp
│       │   └── imu_tf.cpp
│       ├── launch/
│       │   ├── ahrs_data.launch
│       │   └── tf.launch
│       ├── data/                # 传感器数据日志
│       ├── wheeltec_udev.sh
│       ├── CMakeLists.txt
│       └── package.xml
│
├── build/                       # 构建产物（已忽略）
├── devel/                       # 开发空间（已忽略）
├── .gitignore
├── README.md                    # 英文文档
└── README.zh-CN.md              # 中文文档
```

## 配置说明

### IMU 传感器配置

编辑 `launch/Dog.launch` 中的 IMU 参数：

```xml
<param name="port"  value="/dev/ttyUSB0"/>      <!-- 串口 -->
<param name="baud"  value="921600"/>             <!-- 波特率 -->
<param name="device_type"  value="1"/>           <!-- 坐标系模式 -->
```

### 串口设置

```bash
# 列出可用串口
ls /dev/ttyUSB*

# 测试串口连接
sudo minicom -D /dev/ttyUSB0 -b 921600
```

## FSM 状态

控制系统实现了以下状态：

| 状态 | 描述 |
|-------|-------------|
| **Passive**（被动） | 安全模式，电机力矩为零 |
| **FixedStand**（固定站立） | 固定足端位置的站立姿态 |
| **FreeStand**（自由站立） | 带平衡调整的动态站立 |
| **Trotting**（小跑） | 用于运动的小跑步态 |
| **SwingTest**（摆动测试） | 单腿摆动测试 |
| **StepTest**（跨步测试） | 跨步动作测试 |
| **BalanceTest**（平衡测试） | 平衡控制测试 |
| **move_base**（移动底盘） | 底盘移动控制 |

### 状态转换

状态由 `FSM/FSM.h` 中的 FSM 控制器管理。可通过以下方式触发转换：
- 手柄输入
- ROS 服务调用
- 自主控制逻辑

## 故障排除

### 常见问题

**问题**: 访问 `/dev/ttyUSB0` 权限被拒绝
```bash
sudo usermod -aG dialout $USER
# 注销后重新登录
```

**问题**: IMU 数据未发布
```bash
# 检查串口连接
ls -l /dev/ttyUSB*
# 验证波特率设置
# 检查 IMU 驱动日志
rosrun rqt_console rqt_console
```

**问题**: 编译错误
```bash
# 清理构建
catkin_make clean
catkin_make
```

## 数据文件

工作空间包含轨迹数据文件：
- `foot_trajectory_comparison.csv` - 足端轨迹对比数据
- `jump_trajectory.csv` - 跳跃运动轨迹数据

## 贡献指南

欢迎贡献！请遵循以下准则：

1. Fork 本仓库
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建 Pull Request

### 代码风格
- 遵循 ROS C++ 风格指南
- 使用有意义的变量名
- 为复杂逻辑添加注释
- API 变更时更新文档

## 参考资料

- [Unitree Robotics](https://www.unitree.com/)
- [ROS 文档](http://wiki.ros.org/)
- [四足机器人控制](https://www.unitree.com/technology)
- 《四足机器人控制算法--建模、控制与实践》

## 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件。

## 致谢

- Unitree Robotics 提供的原始控制框架
- FDILINK 提供的 AHRS 传感器驱动
- ROS 社区

## 联系方式

如有问题和支持，请在 GitHub 上提 issue。

---

**最后更新**: 2026-03-13
