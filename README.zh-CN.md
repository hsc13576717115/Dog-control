# Dog-control ROS2 Humble 工作空间

这个分支用于当前香橙派机器上的 ROS2 Humble 迁移，系统环境为 Ubuntu 22.04。

原来的 ROS1 `catkin_ws` 已经改成 ROS2 的 `qr_ws` 工作空间。原来的 `unitree_guide` 包已经改名为 `qr_guide`。这次迁移尽量不改动原有控制逻辑，主要调整的是 ROS 接口层、构建系统、launch 文件和工作空间结构。

## 工作空间结构

建议本机使用下面这个目录名：

```bash
~/qr_ws
```

`src/` 下的主要包：

- `fdilink_ahrs`：官方 ROS2 IMU 驱动包
- `serial_ros2`：ROS2 版 `serial` 包源码目录
- `qr_guide`：由原 `unitree_guide` 迁移而来的 ROS2 控制包

## 当前机器约定

这个分支按当前机器环境准备：

- Ubuntu 22.04
- ROS2 Humble
- Orange Pi 5 Plus
- 本机 Unitree 电机 SDK 路径：

```bash
/home/orangepi/unitree_actuator_sdk-main/unitree_actuator_sdk-main
```

当前直接使用你本机已经下载好的官方 ROS2 包：

- `/home/orangepi/fdilink_ahrs_ROS2`
- `/home/orangepi/serial_ros2`

## 构建依赖

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-evdev \
  libeigen3-dev \
  libboost-all-dev \
  liblcm-dev
```

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/qr_ws
colcon build --symlink-install
source install/setup.bash
```

## 启动

如有需要，先配置 IMU 的 udev 规则：

```bash
sudo bash ~/qr_ws/src/fdilink_ahrs/wheeltec_udev.sh
```

启动整套节点：

```bash
source /opt/ros/humble/setup.bash
source ~/qr_ws/install/setup.bash
ros2 launch qr_guide dog.launch.py
```

常用启动参数：

```bash
ros2 launch qr_guide dog.launch.py \
  imu_serial_port:=/dev/wheeltec_FDI_IMU_GNSS \
  imu_serial_baud:=921600 \
  joy_event_path:=/dev/input/event6
```

## 说明

- `qr_guide` 会订阅 `fdilink_ahrs` 发布的 `/imu`。
- `event2joy.py` 已迁移到 `rclpy`，仍然发布 `/joy`。
- 原来的 ROS1 `Dog.launch` 已替换为 `launch/dog.launch.py`。
- 与工作空间相关的 CSV 输出路径已经改到 `~/qr_ws`。
