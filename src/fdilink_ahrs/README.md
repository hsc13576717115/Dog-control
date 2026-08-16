# fdilink_ahrs

`fdilink_ahrs` 是 FDILink 姿态传感器在本工作区中的 ROS 2 驱动包，负责从串口读取原始数据帧，并发布标准 ROS 2 话题供 `custom_dog_control` 控制器使用。

## 1. 包定位

- ROS 2 package：`fdilink_ahrs`
- 可执行文件：`ahrs_driver_node`
- 辅助可执行文件：`imu_tf_node`
- 主 launch：`launch/ahrs_driver.launch.py`

核心职责：

- 从串口读取 FDILink 数据帧
- 完成帧头、长度、CRC、序号检查
- 解析 IMU / AHRS / INS / GPS 数据
- 发布 `/imu`、`/gps/fix`、`/NED_odometry` 等话题

## 2. 在整体系统中的位置

```text
FDILink IMU
  -> /dev/ttyUSB0
  -> fdilink_ahrs::ahrs_driver_node
  -> /imu
  -> custom_dog_control
```

对于四足主控来说，最关键的是：

- `/imu`

这是 `custom_dog_control` 状态估计和机体姿态输入的直接来源。

## 3. 代码框架

| 文件 | 作用 |
| --- | --- |
| `include/ahrs_driver.h` | 驱动节点类定义、参数、发布器、串口对象 |
| `src/ahrs_driver.cpp` | 主解析循环、CRC 检查、话题发布 |
| `include/fdilink_data_struct.h` | 原始帧结构与数据包结构定义 |
| `include/crc_table.h` / `src/crc_table.cpp` | CRC8 / CRC16 / CRC32 查表实现 |
| `launch/ahrs_driver.launch.py` | 驱动节点启动入口 |
| `src/imu_tf.cpp` / `launch/imu_tf.launch.py` | 可选的 IMU TF 广播辅助节点 |

## 4. 运行流程

驱动的核心处理逻辑：

1. 打开串口。
2. 持续读取字节流。
3. 检查帧头 `FRAME_HEAD`。
4. 识别数据类型和长度。
5. 校验 CRC8 和 CRC16。
6. 检查序号是否丢帧。
7. 根据数据类型解析出结构体。
8. 发布对应 ROS 2 话题。

## 5. 主要参数

`ahrs_driver_node` 常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `if_debug_` | `false` | 是否打印详细调试信息 |
| `serial_port_` | `/dev/ttyUSB0` | IMU 串口设备 |
| `serial_baud_` | `921600` | 串口波特率 |
| `device_type_` | `1` | 数据坐标变换模式 |
| `imu_topic` | `/imu` | IMU 输出话题 |
| `imu_frame_id_` | `gyro_link` | IMU 消息 frame_id |
| `mag_pose_2d_topic` | `/mag_pose_2d` | 磁航向输出 |
| `Euler_angles_topic` | `/euler_angles` | 欧拉角输出 |
| `Magnetic_topic` | `/magnetic` | 磁力计输出 |
| `gps_topic` | `/gps/fix` | GPS 输出 |
| `twist_topic` | `/system_speed` | 机体速度输出 |
| `NED_odom_topic` | `/NED_odometry` | NED 里程计输出 |

### 5.1 `device_type_` 说明

- `0`
  直接发布原始姿态与 IMU 数据，不做坐标变换
- `1`
  按当前工程约定做 ROS 侧坐标变换，是本项目默认模式

## 6. 输出接口

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/imu` | `sensor_msgs/msg/Imu` | 姿态、角速度、线加速度 |
| `/mag_pose_2d` | `geometry_msgs/msg/Pose2D` | 航向角 |
| `/euler_angles` | `geometry_msgs/msg/Vector3` | Roll / Pitch / Heading |
| `/magnetic` | `geometry_msgs/msg/Vector3` | 磁力计数据 |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | GPS 经纬高 |
| `/system_speed` | `geometry_msgs/msg/Twist` | 机体系速度 |
| `/NED_odometry` | `nav_msgs/msg/Odometry` | NED 坐标系位置与速度 |

## 7. 启动方式

直接启动驱动：

```bash
cd /home/orangepi/qr_ws
source install/setup.bash
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```

指定串口参数：

```bash
ros2 run fdilink_ahrs ahrs_driver_node --ros-args \
  -p serial_port_:=/dev/ttyUSB0 \
  -p serial_baud_:=921600 \
  -p imu_topic:=/imu
```

## 8. 调试重点

建议优先确认：

1. 串口设备名是否正确。
2. 波特率是否与传感器配置一致。
3. `/imu` 是否稳定输出。
4. 是否出现 CRC 校验失败或序号丢失告警。
5. 坐标变换模式 `device_type_` 是否符合当前系统约定。

## 9. 与 `custom_dog_control` 的关系

对 `custom_dog_control` 来说，这个包最重要的约束是：

- `/imu` 必须持续、稳定、方向一致

如果 IMU 没数据、数据方向不对、或者 frame 约定变化，状态估计和姿态相关控制都会直接受影响。
