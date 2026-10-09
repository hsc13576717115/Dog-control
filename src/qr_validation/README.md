# qr_validation：独立仿真评价

**中文** | [English](README_EN.md)

`scripts/run_precision.py` 冷启动单步测试，独立记录 Gazebo 足端/机身位姿、足端接触与非足端碰撞。控制器不订阅这些评价话题。`run_matrix.py` 对四足与 0/30/50 mm 高度做批量冷启动，保留失败分母与误差样本数。

故障场景：`unreachable`、`cancel`、`replay`、`missed_touchdown`、`stale_imu`。故障用例的 passed 表示正确拒绝或中止，不表示运动成功。完整命令及实测结果见下方文档。未执行的侧碰、滑移、关节反馈过期与求解超时动态注入不能由这些用例代替。

构建、架构与实际验收状态见 [QR 开发文档](../../docs/qr/README.md)。
