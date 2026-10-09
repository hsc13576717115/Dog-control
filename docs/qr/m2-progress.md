# M2 连续换步与困难几何验证

[中文入口](README.md) | [English overview](README_EN.md)

本记录接续 [冻结入口记录](m2-entry.md)。**完整 M2 门槛仍需逐项判定，不把测试套件成功等同于整阶段完成。** 本阶段只运行仿真和离线模型检查，没有真实电机动作。

## 已实现的扩展

1. **有限前进与四脚上台**：M2 独立配置将每次承载确认后的机身终点推进足端位移的四分之一。16 步平地模板与 30/50 mm 整体平台模板分别检查四脚、机身前进、后腿跟进和结束支撑。平台从世界 x=0.3 m 开始，已知地面支撑区在该边界结束；这些是工程试件，不是名义比赛障碍。
2. **先检查整条计划，再逐步执行**：只读 `PreviewFootsteps` 在一个地图/状态快照上预测最多 16 步；后续不可行时，第一步之前就拒绝。执行时仍逐步重新获取状态、规划、action 准入及承载确认。预测不代替实际接触证据。
3. **非实时规划隔离**：单步/整序列预检在独立 executor 串行运行，避免长预检阻塞 IMU 回调。计算期间不持有状态发布/action 互斥锁；批准前复核活动状态、地图版本和忙碌状态。异常返回固定枚举错误。
4. **球足运动学一致性**：M2 显式启用球心滚动补偿；估计器、接触残差与 WBC 使用一致的水平球面接触模型。使用 IMU、关节及模型，没有引入真实足底力或控制侧仿真真值。M1 默认配置不启用此模型。
5. **自碰撞及 IK 修正**：环境保留完整规范碰撞体。原简化机身/大腿碰撞体在站姿已有重叠，自碰撞改用原 URDF 的 CAD 网格，排除同刚体和直接相连的连杆，不排除大腿与固定足端。IK 只返回实际通过 FK 容差检查的关节向量，修复“更新前误差合格、更新并限位后超差仍返回成功”的问题。
6. **协同离线筛查**：`precision_envelope` 支持有限足序、机身平移/姿态搜索、承载前移重心、后续落点、真实质心支撑裕量、场景/自碰撞及零加速度静态接触力 QP。力应用在球足接触点，保持原质量、关节与力矩限制。`precision_replay` 每条离散边再分 10 段检查，仍不是连续碰撞或动态可执行证明。

16 步测试约需 100 秒仿真时间，属于慢速精确验证，不能据此宣称可在 180 秒内完成比赛；动态连续性与时间优化仍属于后续阶段。

参数、单位、帧、接触和线程边界见 [接口说明](interfaces.md)。`max_initial_offset_m` 只限制模板相对初始足位的总行程；单步长度、IK、碰撞、力矩、地图及接触校验不因此绕过。旧模板默认总偏移仍为 100 mm；上台模板显式声明 600 mm。

## 物理求解器收敛检查

保留原 1 ms 时间步、模型、摩擦、控制增益与验收阈值，仅比较 Gazebo ODE 求解迭代次数。长时程的原 50 次与 100 次设置出现累计漂移，不能因为短时 M1 通过就视为长时行走基准。

| 开发比较 | 16 步结果 | 最大落点误差 | 机身前进 |
| --- | --- | --- | --- |
| 200 次 | 通过 | 5.23 mm | 108.20 mm |
| 400 次 | 通过 | 6.16 mm | 104.21 mm |
| 800 次 | 通过 | 6.12 mm | 104.05 mm |

400→800 的前进量差约 0.16 mm，支持本次使用 400 次作为 M2 数值基准；它不是证明任何地形都已数值收敛。原 M1 启动默认仍为 `solver_iterations:=legacy`。上述比较在 IK 收敛修正前执行，最终连续验证应以修正后批次为准，不能混称同一冻结版本。

## 困难几何边界

- **局部双桩**：使用规则第 6 页的 0.2 m 桩体，局部平台边沿到首桩的摆放属于工程假设。搜索包含 7 步和两后腿跟进，不等于六桩覆盖。首次候选在加密复核中暴露 IK 收敛错误，失败保留。修正后重算，7 步候选通过 3401 个加密静态样本检查。
- **沟壑**：规则第 5 页的 0.36 m 净距、0.36 m 平台高，扣除球足半径与 20 mm 边缘裕量。准备后腿站位及最小必要移重心后找到 10 步候选；完整记录包括最后一条后腿过沟及恢复四足支撑。加密静态复核包含 4901 个样本。完整台阶/坡道、入口出口及动态闭环不在这项筛查结论中。
- **高墙**：规则第 5 页的 0.30 m 高、0.08 m 厚、0.80 m 宽。比较前向、先调整后腿、侧身和分段足端路径，保留碰撞/IK 失败阶段。已独立验证一个“前右脚位于墙后地面、其他三脚在墙前”的静态姿态，含 CAD 自碰撞、环境碰撞、关节裕量与静态力矩；**这只能排除该端点绝对不可达的说法，不能证明从接近状态存在完整路径，更不能证明整机能过墙**。有限搜索失败不作机械不可能的结论。

搜索记录保存各脚目标、场景盒、姿态/关节序列、接触集合、采样数与失败分类；所有输入仍是已知工程几何。候选附 `dynamic_execution_certified=false`、`continuous_collision_certified=false`；姿态是否兼容当前控制限幅单独记录。当前 action 不支持直接接收这些机身姿态轨迹，不能把离线 YAML 当作可直接下发的运控计划。

## 可复现测试

先按 [构建说明](README.md) 编译并 source。所有输出目录必须不存在。

```bash
# 四足连续走、整机上台，各 3 次；中途支撑丢失；未来步骤拒绝；M1 定向回归
python3 tools/validate_qr.py --suite m2-continuous \
  --output artifacts/qr/m2-continuous

# 单次完整 50 mm 上台，16 步，要求机身前进至少 0.5 m
python3 src/qr_validation/scripts/run_sequence.py \
  --template src/qr_planning/config/platform_50mm.yaml \
  --control-config src/custom_dog_control/config/precision_m2.yaml \
  --surface-mode platform --height .05 --solver-iterations 400 \
  --minimum-body-advance .50 --output artifacts/qr/platform50
```

离线工具只计算模型，不连接电机：

```bash
ros2 run custom_dog_control precision_envelope \
  external/model_ws/src/custom_dog_description/urdf/custom_dog.urdf \
  src/custom_dog_control/config/precision_model.yaml \
  src/custom_dog_control/config/precision_control.yaml \
  src/custom_dog_control/config/nmpc/task.info \
  src/qr_course/config/rules.yaml /tmp/qr-gap-new.yaml \
  --followup-prepared gap_front_leg 0.240000_0.200000

ros2 run custom_dog_control precision_replay \
  external/model_ws/src/custom_dog_description/urdf/custom_dog.urdf \
  src/custom_dog_control/config/precision_model.yaml \
  src/custom_dog_control/config/precision_control.yaml \
  src/custom_dog_control/config/nmpc/task.info \
  /tmp/qr-gap-new.yaml gap_front_leg 0.240000_0.200000 /tmp/qr-gap-replay-new.yaml
```

扫描是有界离散搜索，可耗时数分钟至更久；`*.partial` 只表示已完成部分变体，不代表整个进程成功。工具返回 0 表示报告生成成功，**是否找到候选必须读取 `complete_candidate`**。加密复核则以 `passed` 决定退出码。

## 尚需完成的完整门槛

- 完成修正后重复动态测试、未来步骤拒绝及中途异常的统一结果；不能沿用 IK 修正前结果冒充回归。
- 桩、沟候选通过独立加密复核，并明确完整入口/出口、前后腿与运行接口的边界。
- 高墙完成有限步序与机身/足端联合路径连接，定位后腿/机身通过阶段的阻塞；尚无完整可执行候选时不报告困难障碍通过。
- 将协同参考接入有容量/时效限制、带接触事件的执行契约后，进行相应动态验证；不能绕开 action 审核直接回放关节角。

M3 的名义障碍完整闭环与扰动、M4 感知、M5 比赛任务、M6 真机/NX、M7 整场计时均未执行。
