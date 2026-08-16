# ROS 2 source packages

该目录由 `colcon` 发现 ROS 2 包，不使用 ROS 1/catkin 顶层构建文件。

- `custom_dog_control`：NMPC、WBC、状态机和 ros2_control 插件。
- `fdilink_ahrs`：IMU 驱动。
- `serial_ros2`：源码目录，对外 ROS 包名为 `serial`。

机器人模型来自外部 underlay 中的 `custom_dog_description` 包。

`unitree_guide` 是历史参考代码，不属于当前 ROS 2 控制运行时。
