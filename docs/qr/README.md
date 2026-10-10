# QR 四足精确越障开发

**中文** | [English](README_EN.md)

开发分支：`RC2027`。主工程为 Dog-control，沿用 ROS 2 Humble、Gazebo、Pinocchio、现有硬件接口及 WBC。HIM 训练与 RL 部署仍在独立仓库。

## 当前硬件前提

机器人只有 **IMU 与关节反馈，没有足底接触传感器**。当前精确模式使用 `q/dq/effort` 与刚体模型估计接触，联合足端速度、载荷迟滞和时间确认。它不是足底力测量；没有有效的关节 effort 或模型不可靠时，不能仅凭静止关节认定承载。真实 effort 的零偏、方向、传动换算和动态误差尚待标定。

首期采用已知局部支撑面，不声称已经有雷达、相机、定位或感知建图。Gazebo 足底接触与机身/足端真值只供 `/evaluation/*` 独立评价，控制器不订阅这些话题。

**精确模式默认关闭，且代码拒绝在真实硬件模式中开启。** 本阶段不驱动真实电机。

## 软件边界

| 模块 | 已加入的职责 |
| --- | --- |
| `custom_dog_control` | 同一个控制器实例内增加精确 WBC、接触估计、单步执行及多高度状态估计；原速度入口保留 |
| `qr_interfaces` | ROS 2 消息、PlanFootsteps/PreviewFootsteps 服务、ExecuteFootsteps 动作 |
| `qr_planning` | 凸支撑区、参考函数和 C++ 有限模板客户端；模型预检在控制器独立非实时规划线程执行 |
| `qr_course` | 规则配置、原始 XML 差异检查、M1 目标面/M2 低平台 SDF 与同源已知支撑面 |
| `qr_bringup` | 仅仿真的完整碰撞单步启动入口 |
| `qr_simulation` | Gazebo 物理步接触滑移积分，只供独立评价 |
| `qr_validation` | 独立真值评价、冷启动和机器可读日志 |

`qr_perception`、`qr_mission` 尚未实现，不创建空包冒充功能。规则输入与工程假设见 [规则配置](../../src/qr_course/config/rules.yaml)，已有 XML 的尺寸差异见 [场景审计](scene-audit.json)。原始 PDF 和 XML 不改写。

MID360 与 FAST-LIO ROS 2 已从指定仓库提取到独立依赖工作区；
固定来源和后续接入注意事项见 [感知依赖说明](perception-source.md)。这不改变当前无雷达输入的 M1 验收条件。

## 构建与单步测试

在 Dog-control 根目录运行；第一次安装需先按根 README 构建 OCS2 与模型 underlay。

```bash
source /opt/ros/humble/setup.bash
source external/ocs2_ws/install/local_setup.bash
source external/model_ws/install/local_setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --symlink-install \
  --packages-up-to custom_dog_control qr_bringup qr_validation \
  --executor sequential --cmake-args -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=OFF
source install/local_setup.bash
ctest --test-dir build/custom_dog_control --output-on-failure
ctest --test-dir build/qr_planning --output-on-failure
python3 src/qr_validation/scripts/run_precision.py \
  --height 0.0 --foot 0 --output artifacts/qr/trial-001
```

`--foot 0/1/2/3` 对应 FR/FL/RR/RL；高度支持 `0/0.03/0.05`。输出目录必须不存在，防止覆盖失败证据。增加 `--gui` 可查看画面。测试使用独立 ROS domain 与 Gazebo master，启动器始终选择仿真插件。

仅启动场景：

```bash
ros2 launch qr_bringup precision_step.launch.py height:=0.03 gui:=true
```

M1 从仿真初始站立配置启动，有一个姿态保持和接触确认阶段。**没有完成趴姿起身验收**，也不等于整场比赛自主启动流程。

## 使用限制

- 当前 ExecuteFootsteps 只接受经过同一实例 PlanFootsteps 预检的单步计划；过期、改写或未预检的计划拒绝执行。
- 支持水平已知支撑面，0/30/50 mm 测试高度；不支持任意斜面、在线接触组合搜索或跳跃。
- 规划含轨迹 IK、关节裕量、已提供的矩形平台及地面碰撞与 WBC 支撑/力矩可行性预检；这不是通用全环境碰撞规划器。
- M2 支持带机身前进的有限步序和 30/50 mm 四足平台转移，需使用独立 M2 配置；名义桩、墙、沟尚未形成动态闭环验收。见 [M2 进展](m2-progress.md)。感知地图和返航仍按 [开发路线](roadmap.md) 实施。
- 现有 WBC 本身包含动态内存分配；不能声称新增模式已是硬实时实现或已在 NX 验证。
- 控制周期 1000 Hz 是配置，不是传感器新鲜反馈频率。接触估计力和置信度必须与独立真值分开评价。

实际验证结果见 [验证记录](validation.md)。不能将编译或单元测试通过写成 M1 仿真验收通过。

M1 收尾新增采样时间契约、持续摆腿受阻检测、支撑失效退出和独立滑移评价，详见 [收尾记录](m1-closeout.md)。

## 批量与故障测试

```bash
python3 src/qr_validation/scripts/run_matrix.py \
  --trials-per-foot 25 --jobs 2 --output artifacts/qr/matrix-100
python3 src/qr_validation/scripts/run_precision.py \
  --scenario cancel --output artifacts/qr/cancel
python3 src/qr_validation/scripts/run_precision.py \
  --scenario missed_touchdown --height 0.03 --output artifacts/qr/missed-contact
python3 src/qr_validation/scripts/run_precision.py \
  --scenario stale_imu --output artifacts/qr/stale-imu
```

其他场景为 `unreachable`（抬腿前拒绝）和 `replay`（拒绝重复计划）。
测试默认不用画面，`--jobs 1` 可串行执行；两个并行实例使用独立 ROS domain 和 Gazebo master。
成功率分母包含所有启动失败、拒绝和中止。落点误差统计针对已完成动作，并明确记录样本数；
不能把缺失数据填成零误差。`step_matrix_accepted` 只表示已知场景的单步矩阵，不代表完整 M1 故障覆盖或比赛验收。

启动过程在暂停的 Gazebo 中逐步推进并确认控制器激活，然后恢复连续物理仿真，
避免固定延时解暂停造成未受控下落。独立评价除足端接触外，还检查腿部及机身碰撞。
每份结果保存源码/模型/配置哈希、场景版本、随机种子与原始 trace；旧失败记录不覆盖。
种子变化不等于已经做了质量、摩擦、传感器噪声或地形几何的域随机化。

## 模块化维护与统一验收

具体方案见 [模块化改造](refactoring.md)，实测结果见 [改造验证](refactoring-results.md)。
`PrecisionRuntime` 只组织 `PrecisionPlanner`、`PrecisionExecutionCore`、`PrecisionRosAdapter`。
规划器和执行核心不依赖 ROS，独占各自模型缓存；精确模式直接使用公共 `RobotModel`，
不初始化 NMPC 优化问题。公共接触类型位于 `control/ContactTypes.hpp`。

三类配置分别为 `precision_model.yaml`、`precision_control.yaml`、
`qr_validation/config/acceptance.yaml`。允许范围及迟滞/时序关系在配置时检查。
新增数值状态码需要重新构建消息消费端，含义见 [接口契约](interfaces.md)。

构建并 source 工作区后，统一使用：

```bash
python3 tools/validate_qr.py --suite unit --output artifacts/qr/unit-001
python3 tools/validate_qr.py --suite regression --output artifacts/qr/regression-001
```

输出目录必须未存在。`unit` 包含 CTest 与 Python；`smoke` 为四足 50 mm；
`faults` 为现有五类异常；`regression` 为四足×三高度＋五类异常＋原速度运动；
`matrix` 单独执行 100 次矩阵。各套件先检查依赖，写统一 `summary.json` 和分项日志；
失败非零退出。未涵盖的动态故障列在报告中，不因套件通过而宣称完整验收。

M1 收尾结果为 99/100；M2 有限连续模板与机械筛查正在推进，完整阶段尚未通过，见 [M2 记录](m2-entry.md)。

M2 本轮已通过 9 次连续试验（144 步）及最终 25 项回归；高墙完整步序和名义障碍协同执行仍未通过。详见 [结果与未通过门槛](m2-progress.md)。

[新增协同参考接口及阶段限制](coordinated-reference.md)。
