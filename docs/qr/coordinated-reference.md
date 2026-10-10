# 有界协同参考：接口与验证范围

[中文入口](README.md) | [English overview](README_EN.md)

本功能只供 Gazebo 已知几何实验；不是已完成的 M2 名义障碍通行器。控制端仍只有 IMU、关节位置/速度及 effort 残差，没有足底传感器或仿真接触真值输入。

## 执行契约

`precision_coordinated.yaml` 显式启用 `coordinated_enabled=1`。M1 与原 M2 配置保持关闭。需要同时重建 `qr_interfaces` 及所有调用方；`ExecutionStatus.PROTOCOL_VERSION` 为 2。

`PlanFootsteps` 可附最多 192 个 `ReferenceKnot` 和 `lift_index`：

- `time_from_start`：秒，首节点为 0，严格递增，每段至少两个预检采样周期。
- 请求的 `body`：相对准入快照机身位置的偏移，单位米；`euler_zyx`：相对此快照的 yaw/pitch/roll 偏移，单位弧度。首节点偏移应为零。
- `foot`：移动足球心在 `odom` 下的绝对位置，单位米。抬腿节点及之前保持原支撑位置；只吸收准入时允许的估计/参考差异。
- 批准后的 `Footstep.trajectory` 全部转换成 `odom` 下的绝对参考，供审阅与 action 严格一致性校验。不能把请求偏移格式原样下发 action。
- `lift_index` 将轨迹分成四足移重心与单腿摆动两段。默认触地确认后恢复到原有 `body_follow_ratio` 给出的机身目标；`keep_terminal_body=true` 可显式保留轨迹末端机身位置，仍须通过完整预检。

每段为端点速度、加速度为零的五次插值。姿态参考同时提供 ZYX 坐标导数和二阶导数；它们不是世界角速度。IK 使用基座完整六维速度补偿，WBC 接收姿态前馈。此实现适用于缓慢有限步序，节点间停车会影响通行时间，不是高速平滑轨迹优化器。

控制线程使用固定容量对象；只复制有效节点，不进行动态轨迹分配。时长、速度与姿态边界在非实时预检时校验。当前运动学/碰撞检查仍为离散采样，不宣称连续碰撞保证或硬实时 WCET。

`SHIFT → SWING → CONTACT_CONFIRM → RESTORE` 的接触事件逻辑保留。轨迹时间结束不会直接证明踩实；缺乏承载证据时仍执行有界探测/超时中止。取消会冻结参考并清除线速度、角坐标速度及加速度；支撑丢失继续进入现有故障链路。

整序列 `PreviewFootsteps` 目前只支持原有限目标模板，**显式拒绝包含协同节点的请求**。测试客户端的初次整序列预检仅覆盖原目标模板；每步协同曲线另外通过 `PlanFootsteps` 验证，不能将两者说成完整协同序列预检。名义桩/沟离线 YAML 也不能直接交给电机或 action。

## 已知初始化试件

`precision_step.launch.py` 可配对接收 `fixture_file` 和 `startup_file`。前者为 `qr-coordinated-local-v1` 已知轴对齐盒体定义，统一生成物理场景和已知支撑区域；后者明确 FR/FL/RR/RL、每腿 hip/thigh/calf 顺序的 12 个初始关节角及 ZYX 姿态。两者都是实验条件，不是感知输出。

`precision_model.yaml` 增加 `initial_support_height_m`（默认 0），用于已知初始平台高度。机身高度必须高于支撑面与球足半径之和；不可把未知地面配置成已知面。之后仍由估计接触和锚点维护高度约束。一般倾斜支撑、多层地图及从任意姿态自主初始化尚未实现。

## 复现

完成构建、source 后：

```bash
python3 tools/validate_qr.py --suite m2-coordinated \
  --output artifacts/qr/coordinated-new
```

该入口包括单元测试、平地与 30/50 mm 连续序列、未来步骤拒绝、中途支撑丢失、四足 M1 兼容测试及已知高平台初始化测试。测试门槛未修改，结果保留源码/模型/配置/二进制哈希。`stage_accepted` 保持 false，不能凭这个套件替代高墙完整步序及名义障碍闭环验收。

平地序列额外使用 `--exercise-body-orientation`，在摆动中间节点增加 0.03 rad 偏航与 5 mm 机身抬高，检查姿态参考链路。平台序列使用原方向与规划器给出的机身目标，不额外插入偏航。前一次在平台上也附加此动作的试验出现支撑足球心位移超过 10 mm，按失败保留；虽然材料接触点滑移很小，仍不降低球心位移验收门槛。两个试验输入不同，不能据后续直行试验宣称平台附加偏航已通过。

本次最终结果在 [协同参考测试记录](coordinated-results.json) 中记录；高墙、局部桩阵和沟壑的完整阶段门槛仍见 [M2 进度](m2-progress.md)。

## 本次实测结果

`m2-coordinated-final-r2` 的 **14/14 测试入口通过**，包含四个包共 16 个 CTest 入口。平地、30 mm、50 mm 各完成一次 16 步序列；落点误差 P95 分别为 6.11、7.69、7.57 mm，最大支撑足球心位移分别为 6.97、9.44、9.74 mm。50 mm 项距离 10 mm 上限较近，尚未做噪声、摩擦、延迟或尺寸扰动统计。

未来步骤无效时在执行前拒绝；完成四步后施加有限足端外力，实际由 `total_torque_limit` 中止第五步，没有报告整个序列成功。这证明该扰动下的力矩保护与序列停止，不等同于单独验证所有接触丢失分支。四足旧单步入口均通过；50 mm 已知初始平台额外完成四步，P95 6.04 mm。未运行真实电机。
