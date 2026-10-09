# qr_bringup：四足仿真启动

**中文** | [English](README_EN.md)

`launch/precision_step.launch.py` 启动完整碰撞模型、精确控制器与已知支撑面。`scripts/activate_paused.py` 在暂停状态下逐步推进 Gazebo，确认控制器接管后才连续运行。

```bash
ros2 launch qr_bringup precision_step.launch.py height:=0.03 gui:=true
```

默认无界面。`reported_height` 与 `imu_fault_relay` 只用于验证器故障注入，普通测试保持默认值。精确模式拒绝真实硬件，原速度模式使用原有启动入口。

构建、架构与实际验收状态见 [QR 开发文档](../../docs/qr/README.md)。
