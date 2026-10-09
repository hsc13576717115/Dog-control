# qr_validation：独立仿真评价

**中文** | [English](README_EN.md)

`scripts/run_precision.py` 冷启动单步测试，独立记录 Gazebo 足端/机身位姿、足端接触与非足端碰撞。控制器不订阅这些评价话题。`run_matrix.py` 对四足与 0/30/50 mm 高度做批量冷启动，保留失败分母与误差样本数。

故障场景：`unreachable`、`cancel`、`replay`、`missed_touchdown`、`stale_imu`。故障用例的 passed 表示正确拒绝或中止，不表示运动成功。完整命令及实测结果见下方文档。未执行的侧碰、滑移、关节反馈过期与求解超时动态注入不能由这些用例代替。

构建、架构与实际验收状态见 [QR 开发文档](../../docs/qr/README.md)。

统一入口（在仓库根目录且已加载构建环境）：

```bash
python3 tools/validate_qr.py --suite regression --output artifacts/qr/regression-001
```

`unit/smoke/faults/regression/matrix` 各自范围见根文档。Python 场景与协议测试现已注册
CTest。独立验收阈值在 `config/acceptance.yaml`，每次单步/矩阵报告保存实际阈值和哈希。
