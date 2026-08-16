# serial_ros2

`serial_ros2` 在这个工作区中承担“串口基础库”的角色，主要给 `fdilink_ahrs` 提供串口打开、配置、读写和超时控制能力。

## 1. 包定位

- 目录名：`serial_ros2`
- 导出的 ROS 2 package 名：`serial`
- 类型：第三方基础依赖库

当前在 `qr_ws` 中的主要用途：

- 被 `fdilink_ahrs` 依赖，用于 IMU 串口通信
- 不直接承载四足控制逻辑

## 2. 建议的维护方式

这个包更适合视作“基础组件”而不是“业务代码”：

- 一般不建议为了业务需求直接大改它的底层实现
- 如果只是上层驱动逻辑变化，优先改 `fdilink_ahrs`
- 如果以后要升级串口库版本，建议尽量保持这个目录改动最小

## 3. 对外能力

这个库主要提供：

- 串口端口打开与关闭
- 波特率、校验位、停止位、流控设置
- 阻塞/非阻塞读写
- 超时控制
- Linux / Windows 差异封装

## 4. 在本工作区中的依赖关系

```text
fdilink_ahrs
  -> serial
  -> /dev/ttyUSB0
  -> FDILink IMU
```

## 5. 编译

在工作区根目录执行：

```bash
cd /home/orangepi/qr_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select serial
```

## 6. 当前结论

对于这个工作区来说，更重要的是把它当成“稳定依赖”来使用：

- `custom_dog_control` 关注控制逻辑
- `fdilink_ahrs` 关注协议解析和 ROS 话题
- `serial` 只负责提供可靠的串口基础能力
