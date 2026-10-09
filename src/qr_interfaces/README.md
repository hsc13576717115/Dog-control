# qr_interfaces：四足控制接口

**中文** | [English](README_EN.md)

定义状态、接触估计、支撑区域、落足计划与执行结果。`PlanFootsteps` 返回预检计划，`ExecuteFootsteps` 执行已批准的单步。足序为 FR/FL/RR/RL，目标为 odom 下足心坐标，ROS 四元数为 XYZW。

没有足底传感器：`force_valid=false`；动力学残差只写入 `estimated_force`，并保留独立有效标记。`ObstacleArray` 和 `MissionStatus` 是后续契约，当前没有任务计分或感知节点。

构建、架构与实际验收状态见 [QR 开发文档](../../docs/qr/README.md)。
