# Dog-control ROS2 Humble Workspace

This branch is the ROS2 Humble migration for Ubuntu 22.04 on the local Orange Pi machine.

The original ROS1 `catkin_ws` layout has been converted into a ROS2 `qr_ws` workspace. The original `unitree_guide` package is now named `qr_guide`. The control logic is kept as close to the original code as possible; the main changes are the ROS middleware layer, build system, launch files, and workspace structure.

## Workspace Layout

Recommended local path:

```bash
~/qr_ws
```

Main packages inside `src/`:

- `fdilink_ahrs`: official ROS2 IMU driver package
- `serial_ros2`: source folder of the ROS2 `serial` package
- `qr_guide`: ROS2 port of the original quadruped control package

## Local Machine Assumptions

This branch is prepared for the current machine:

- Ubuntu 22.04
- ROS2 Humble
- Orange Pi 5 Plus
- Local Unitree actuator SDK path:

```bash
/home/orangepi/unitree_actuator_sdk-main/unitree_actuator_sdk-main
```

The following local official ROS2 packages are used directly:

- `/home/orangepi/fdilink_ahrs_ROS2`
- `/home/orangepi/serial_ros2`

## Build Dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-evdev \
  libeigen3-dev \
  libboost-all-dev \
  liblcm-dev
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/qr_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch

Prepare the IMU udev rule if needed:

```bash
sudo bash ~/qr_ws/src/fdilink_ahrs/wheeltec_udev.sh
```

Run the full stack:

```bash
source /opt/ros/humble/setup.bash
source ~/qr_ws/install/setup.bash
ros2 launch qr_guide dog.launch.py
```

Useful launch arguments:

```bash
ros2 launch qr_guide dog.launch.py \
  imu_serial_port:=/dev/wheeltec_FDI_IMU_GNSS \
  imu_serial_baud:=921600 \
  joy_event_path:=/dev/input/event6
```

## Notes

- `qr_guide` subscribes to `/imu` from `fdilink_ahrs`.
- `event2joy.py` has been migrated to `rclpy` and still publishes `/joy`.
- The old ROS1 launch file has been replaced by `launch/dog.launch.py`.
- The workspace-specific CSV output paths now point to `~/qr_ws`.
