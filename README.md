# Dog Control System

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Platform](https://img.shields.io/badge/platform-Orange%20Pi-orange.svg)](https://www.orangepi.org/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

A quadruped robot control system based on Unitree Robotics framework, implementing advanced gait control and state machine management for robotic dog applications.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [FSM States](#fsm-states)
- [Contributing](#contributing)
- [License](#license)

## Overview

This project provides a comprehensive control system for quadruped robots, specifically designed for Unitree robotic platforms. It implements a finite state machine (FSM) architecture with various gaits and control modes, integrated with IMU sensor fusion for precise motion control.

### Key Components

- **unitree_guide**: Main control package implementing gait generation, state management, and robot control
- **fdilink_ahrs**: IMU/AHRS driver package for sensor data acquisition

## Features

### Motion Control
- Multiple gait patterns: Trotting, Swing Test, Step Test
- Balance testing mode
- Fixed and free standing modes
- Base movement control
- Passive mode for safe operation

### Sensor Integration
- FDILINK AHRS IMU sensor driver
- Real-time attitude estimation
- GPS data fusion
- Magnetic field sensing
- Velocity and odometry tracking

### Control Architecture
- Finite State Machine (FSM) design
- Modular gait generator
- Feet trajectory planning
- Joint-level control interface

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Dog Control System                     │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   IMU       │    │  Controller │    │   Gait      │     │
│  │   Driver    │───▶│   Node      │───▶│  Generator  │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│         │                  │                  │             │
│         ▼                  ▼                  ▼             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │ Sensor      │    │    FSM      │    │  Feet       │     │
│  │ Topics      │    │  Manager    │    │  Trajectory │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## Hardware Requirements

### Minimum Requirements
- **Platform**: Orange Pi (RK3588) or similar ARM64 platform
- **RAM**: 4GB+ recommended
- **Storage**: 10GB+ free space

### Peripherals
- FDILINK AHRS IMU sensor
- Unitree quadruped robot (A1/Go1 series)
- USB serial connection for IMU
- Optional: GPS module

## Software Dependencies

### Core Dependencies
- **ROS**: Noetic Ninjemmys
- **Catkin**: Build system
- **C++ Standard**: C++11 or later

### ROS Packages
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

### System Packages
```bash
# Ubuntu/Debian
sudo apt install -y \
    cmake \
    python3-catkin-tools \
    libeigen3-dev
```

## Installation

### 1. Clone the Repository

```bash
cd ~/catkin_ws/src
git clone git@github.com:hsc13576717115/Dog-control.git
```

### 2. Install ROS Dependencies

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
cd ~/catkin_ws
catkin_make
# or using catkin tools
catkin build
```

### 4. Source the Workspace

```bash
source ~/catkin_ws/devel/setup.bash
# Add to ~/.bashrc for automatic sourcing
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 5. Configure USB Permissions (for IMU)

```bash
# Create udev rule for FDILINK AHRS
sudo bash ~/catkin_ws/src/fdilink_ahrs/wheeltec_udev.sh
# Or manually add user to dialout group
sudo usermod -aG dialout $USER
```

## Usage

### Launch the Control System

```bash
roslaunch unitree_guide Dog.launch
```

This will start:
1. AHRS/IMU driver node
2. Event-to-joystick converter
3. Main robot control node

### Available ROS Topics

#### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/imu` | `sensor_msgs/Imu` | IMU sensor data |
| `/mag_pose_2d` | `geometry_msgs/Pose2D` | Magnetic heading |
| `/euler_angles` | `geometry_msgs/Vector3` | Euler angles (roll, pitch, yaw) |
| `/magnetic` | `geometry_msgs/Vector3` | Magnetic field strength |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS position data |
| `/system_speed` | `geometry_msgs/Twist` | Body frame velocity |
| `/NED_odometry` | `nav_msgs/Odometry` | NED frame odometry |

### Manual Control

The system supports joystick control through the event interface. Ensure your input device is properly configured.

## Project Structure

```
catkin_ws/
├── src/
│   ├── unitree_guide/           # Main control package
│   │   ├── include/
│   │   │   ├── FSM/             # Finite State Machine states
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
│   │   │   ├── Gait/            # Gait generation
│   │   │   │   ├── GaitGenerator.h
│   │   │   │   ├── WaveGenerator.h
│   │   │   │   └── FeetEndCal.h
│   │   │   ├── control/         # Control algorithms
│   │   │   ├── interface/       # Robot interface
│   │   │   ├── common/          # Common utilities
│   │   │   ├── message/         # Message definitions
│   │   │   └── thirdParty/      # Third-party libraries
│   │   ├── src/                 # Source code
│   │   ├── launch/              # Launch files
│   │   │   └── Dog.launch
│   │   ├── scripts/             # Python scripts
│   │   │   └── event2joy.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── fdilink_ahrs/            # IMU driver package
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
│       ├── data/                # Sensor data logs
│       ├── wheeltec_udev.sh
│       ├── CMakeLists.txt
│       └── package.xml
│
├── build/                       # Build artifacts (gitignored)
├── devel/                       # Development space (gitignored)
├── .gitignore
└── README.md
```

## Configuration

### IMU Sensor Configuration

Edit the IMU parameters in `launch/Dog.launch`:

```xml
<param name="port"  value="/dev/ttyUSB0"/>      <!-- Serial port -->
<param name="baud"  value="921600"/>             <!-- Baud rate -->
<param name="device_type"  value="1"/>           <!-- Coordinate frame mode -->
```

### Serial Port Setup

```bash
# List available serial ports
ls /dev/ttyUSB*

# Test serial connection
sudo minicom -D /dev/ttyUSB0 -b 921600
```

## FSM States

The control system implements the following states:

| State | Description |
|-------|-------------|
| **Passive** | Safe mode with zero motor torque |
| **FixedStand** | Standing position with fixed foot placement |
| **FreeStand** | Dynamic standing with balance adjustment |
| **Trotting** | Trot gait for locomotion |
| **SwingTest** | Individual leg swing testing |
| **StepTest** | Stepping motion test |
| **BalanceTest** | Balance control testing |
| **move_base** | Base movement control |

### State Transitions

States are managed through the FSM controller in `FSM/FSM.h`. Transitions can be triggered via:
- Joystick input
- ROS service calls
- Autonomous control logic

## Troubleshooting

### Common Issues

**Issue**: Permission denied accessing `/dev/ttyUSB0`
```bash
sudo usermod -aG dialout $USER
# Log out and log back in
```

**Issue**: IMU data not publishing
```bash
# Check serial connection
ls -l /dev/ttyUSB*
# Verify baud rate setting
# Check IMU driver logs
rosrun rqt_console rqt_console
```

**Issue**: Build errors
```bash
# Clean build
catkin_make clean
catkin_make
```

## Data Files

The workspace includes trajectory data files:
- `foot_trajectory_comparison.csv` - Foot trajectory comparison data
- `jump_trajectory.csv` - Jump motion trajectory data

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Code Style
- Follow ROS C++ style guide
- Use meaningful variable names
- Add comments for complex logic
- Update documentation for API changes

## References

- [Unitree Robotics](https://www.unitree.com/)
- [ROS Documentation](http://wiki.ros.org/)
- [Quadruped Robot Control](https://www.unitree.com/technology)
- 《四足机器人控制算法--建模、控制与实践》

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Unitree Robotics for the original control framework
- FDILINK for the AHRS sensor driver
- The ROS community

## Contact

For questions and support, please open an issue on GitHub.

---

**Last Updated**: 2026-03-13
