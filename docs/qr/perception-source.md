# MID360 与 FAST-LIO ROS 2 来源及接入边界

[中文入口](README.md) | [English overview](README_EN.md)

## 固定来源

按项目所有者指定，使用 [ROBOCON_NBUT_R2](https://github.com/hsc13576717115/ROBOCON_NBUT_R2/tree/300d07241372180c5267a129e80ad9a3410e2997)，
固定提交 `300d07241372180c5267a129e80ad9a3410e2997`，不跟随远端分支自动更新。

| 上游路径 | ROS 包名 | 用途 |
| --- | --- | --- |
| `src/livox_ros_driver2` | `livox_ros_driver2` | MID360 点云和雷达内置 IMU 驱动 |
| `src/FAST_LIO_ROS2` | `fast_lio` | 雷达惯性里程计，ROS 2/ament 版本 |

源码保留原版权与许可证；Livox 有 MIT 及第三方声明。FAST-LIO 的 `package.xml`
写 BSD，但同目录 `LICENSE` 为 GPL v2 文本，两者不一致，不能将整个依赖标为 BSD。
发布时需按实际文件范围核对许可，不能删除原声明。

## 获取与构建边界

在 Dog-control 根目录执行：

```bash
./tools/fetch_perception.sh
```

脚本只提取上述两个目录到 `external/perception_ws/src`，保留全部包内许可证，
并记录 `SOURCE.yaml`。已有目录时拒绝覆盖，避免破坏本地标定或修改。
依赖副本不提交，脚本和固定版本纳入主仓库；主工作区不自动发现或启动它们。

准备 Humble、PCL、ament 依赖和 Livox-SDK2 后，可独立构建：

```bash
source /opt/ros/humble/setup.bash
cd external/perception_ws
export LIVOX_SDK2_DIR=/absolute/path/to/Livox-SDK2
colcon build --base-paths src --packages-up-to fast_lio --executor sequential \
  --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DBUILD_TESTING=OFF
```

SDK 需具有 `include/livox_lidar_api.h` 和 `build/sdk_core/liblivox_lidar_sdk_shared.so`。
其版本尚未锁定，不宣称整条感知链路已经可复现构建。
不要运行上游 `livox_ros_driver2/build.sh`：它会删除相对路径下的 build/install 目录。
上游 SDK 安装脚本会执行 apt/sudo 并拉取可变版本；本次没有执行。

## 已核对的接口与必须调整的内容

- 驱动使用 `xfer_format=1`，与 FAST-LIO 的 Livox 自定义消息匹配，输入为
  `/livox/lidar` 和 `/livox/imu`。这里的 IMU 是雷达内置 IMU，不应直接替换为机身 IMU。
- 原驱动配置主机 IP 为 `192.168.1.5`，雷达 IP 为 `192.168.1.163`；这是原机器人配置，
  需要按四足实际接线修改，尚未连接或发现设备。
- FAST-LIO 的 `mid360.yaml` 当前设置 0.5 m 近场滤除、0.5 m 地图/表面降采样，
  且点云发布关闭。这是已有定位配置，不能直接用于厘米级落足地图。
- `extrinsic_T/R` 是雷达与其 IMU 的关系，不是雷达到四足机身的外参；
  `base` 到雷达的刚性外参、时间同步与延迟需另行标定。
- 当前源码发布 `/Odometry` 和 `camera_init → body` TF。
  接入时必须明确雷达 IMU `body` 与机器人 `base` 的关系，隔离 TF 发布权，
  不得把这个 `body` 直接当作机器人的机身帧。
- FAST-LIO 本身不能替代比赛全局定位、回环或障碍语义建图。
  后续 `qr_perception` 需要提供置信度、未知区域、支撑面与三维障碍；
  定位修正不能移动控制器 `odom` 中已承载的足端锚点。

当前机器人硬件假设仍为 IMU＋关节，没有足底传感器；新增依赖来源不表示已经安装雷达。
本次仅获取并检查源码，没有启动雷达、运行定位或把定位输出接入控制器。

## English summary

The MID360 driver and ROS 2 FAST-LIO sources are pinned to the owner's repository
at commit `300d07241372180c5267a129e80ad9a3410e2997`. Run `tools/fetch_perception.sh`
to reproduce the isolated `external/perception_ws` source workspace. Existing
workspaces are never overwritten. Neither the driver nor localization has been
hardware-tested. Livox-SDK2 remains a separate, not-yet-pinned build dependency.

The upstream IP addresses, lidar-to-IMU extrinsics, TF names (`camera_init`, `body`),
disabled point-cloud outputs and coarse 0.5 m filtering require deliberate integration.
The current configuration is not a precision foothold map. Preserve the upstream
licenses: FAST-LIO's package metadata and LICENSE disagree (BSD vs GPL v2 text).
