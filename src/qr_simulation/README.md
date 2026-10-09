# QR 仿真评价插件

[English](README_EN.md)

`qr_contact_metrics` 在 Gazebo 每个物理步中累计四足接触点的切向相对速度积分，100 Hz 发布
`/evaluation/contact_metrics`。它属于独立评价链路，不能作为只有 IMU 和关节反馈的控制输入。

- 使用当前接触流形中各点的平均切向速度；考虑双方刚体的线速度与角速度。
- 足心位移会包含球形足滚动；该指标将理想纯滚动与滑动区分开。
- 数值接触、刚性球形足和离散积分仍有误差，不代表真机可变形足底的实际滑移。
- 发布从本次仿真开始的累计值和物理序号；评价器自行记录动作前基线并取差。
- 关节顺序 FR/FL/RR/RL，模型名 `custom_dog`，足端链接名沿用规范 URDF。
- 只用于 Gazebo Classic；没有任何电机、硬件命令或控制话题订阅。

由 `qr_bringup/precision_step.launch.py` 加载；运行 `tools/validate_qr.py` 的仿真套件验证。
