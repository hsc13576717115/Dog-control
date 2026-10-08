# ROS 2 source packages

该目录由 `colcon` 发现 ROS 2 包，不使用 ROS 1/catkin 顶层构建文件。

- `custom_dog_control`：NMPC、WBC、状态机和 ros2_control 插件。
- `fdilink_ahrs`：IMU 驱动。
- `serial_ros2`：源码目录，对外 ROS 包名为 `serial`。

机器人模型来自外部 underlay 中的 `custom_dog_description` 包。

`unitree_guide` 是历史参考代码，不属于当前 ROS 2 控制运行时。

## 工作区与依赖关系

箭头表示依赖包向使用方提供的能力或数据；`serial_ros2` 是目录名，实际包名为 `serial`。

```mermaid
flowchart TB
    subgraph underlay["外部 underlay"]
        description["external/model_ws<br/>custom_dog_description"]
        ocs2["external/ocs2_ws<br/>OCS2：SQP / MRT / 质心模型"]
        ros["ROS 2 Humble<br/>ros2_control / Gazebo / Pinocchio"]
    end
    subgraph workspace["Dog-control/src · colcon 工作区"]
        serial["serial_ros2 目录<br/>ROS 包：serial"] --> imu["fdilink_ahrs<br/>IMU 串口驱动"]
        imu -->|"/imu"| control["custom_dog_control<br/>控制器、NMPC/WBC 适配、状态机、实机插件"]
        vendor["custom_dog_control/third_party<br/>LeggedInterface / legged_wbc / qpOASES"] -->|"编译链接"| control
    end
    description -->|"URDF / 网格"| control
    ocs2 -->|"算法库"| control
    ros -->|"控制框架 / 仿真 / 模型计算"| control
    sdk["外部 Unitree actuator SDK"] -->|"启用实机构建时"| control
    control --> sim["gazebo.launch.py<br/>Gazebo 仿真"]
    control --> real["real.launch.py<br/>电机 SDK 的 RS485 通信"]
    historical["unitree_guide<br/>历史参考，COLCON_IGNORE 排除"]

    classDef package fill:#e8f1ff,stroke:#4775ad,color:#172b4d;
    classDef dependency fill:#e5f5ec,stroke:#39815a,color:#193e2b;
    classDef inactive fill:#f1f1f1,stroke:#999,color:#555;
    class serial,imu,control package;
    class description,ocs2,ros,vendor,sdk dependency;
    class historical inactive;
```

`serial` 服务于 IMU 驱动；电机通信使用 Unitree SDK 的串口实现。
`third_party` 中的算法随主包编译，并非独立 ROS 节点。

整体闭环见 [工作区 README](../README.md#软件结构)，
求解线程、状态维度和 WBC 实现见 [控制包 README](custom_dog_control/README.md#算法链路)。
