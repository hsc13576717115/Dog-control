# ROS 2 source packages

[中文](README.md) | **English**

`custom_dog_control` contains the traditional NMPC/WBC controller, hardware adapter,
and experimental precision tracking. `fdilink_ahrs` and `serial_ros2` provide the
IMU driver and its serial dependency. The robot model is built in a separate underlay.

The `qr_interfaces`, `qr_planning`, `qr_course`, `qr_bringup`, and `qr_validation`
packages add contracts, planning primitives, known test geometry, Gazebo startup,
and independent evaluation. QR precision control currently uses IMU and joint feedback
to estimate contact; there are no sole contact sensors. It is simulation-only.
See [QR development and validation](../docs/qr/README_EN.md).

MID360 and FAST-LIO sources are isolated in `external/perception_ws`; see
[source provenance and integration limits](../docs/qr/perception-source.md).
RL deployment remains in the independent `himloco_custom_dog/deployment` repository.
Historical `unitree_guide` code is excluded from colcon discovery.

- `qr_simulation`: independent contact-slip integration at each physics step; never a controller input.
