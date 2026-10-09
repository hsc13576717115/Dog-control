# QR 模块化改造方案与验收

[中文入口](README.md) | [English](README_EN.md)

基线：`cc62849`。仅改进现有仿真控制框架，不引入 RL、不驱动真实电机。

## 实施顺序与文件边界

1. `LatestSnapshot.hpp` 使用固定三缓冲交换，单一控制线程写，非实时读者串行取得副本。
   控制侧固定次数原子操作、无读者互斥锁；中间版本可丢弃。快照错误使用固定枚举。
   验收包括并发完整性、读者长期持有副本时写者继续、生命周期清空。
2. `PrecisionPlanner` 接收无 ROS 的状态和支撑面快照，独占预检模型/WBC；
   `PrecisionExecutionCore` 独占状态估计、接触观察、执行器及控制 WBC；
   `PrecisionRosAdapter` 负责消息转换、计划准入、action 和发布；
   `PrecisionRuntime` 连接通道、生命周期和调用，不包含规划/控制算法。
   规划器暂放本包，避免让轻量 `qr_planning` 反向依赖控制器。
3. 公共 `control/ContactTypes.hpp` 与 `model/RobotModel` 解耦接触和模型。
   精确模式直接加载并验证 URDF/质心信息，不创建 NMPC 问题或求解器；旧速度模式保持。
4. `precision_model.yaml` 保存足底尺寸和模型校验参数；`precision_control.yaml` 保存
   接触/轨迹/WBC/规划参数；`qr_validation/config/acceptance.yaml` 独立保存验收门槛。
   启动拒绝缺字段、非有限值、非法范围和矛盾组合，运行时不热改参数。
   阶段和错误新增固定数值码，保留文字兼容调试；协议版本和数值映射纳入契约测试。
5. `tools/validate_qr.py` 统一依赖检查、单元测试、仿真 smoke/fault/matrix 入口和 JSON 报告。
   场景 Python 测试注册 CTest。完整矩阵和 Gazebo 不作为每次普通 CTest 的隐式副作用。

## 验证顺序

配置拒绝、公共模型一致性、线程交换和接口契约 → 构建 → CTest/Python → 四足 0/30/50 mm
独立冷启动 → 已有取消/不可达/重放/漏触地/过期 IMU → 旧速度运动回归。
默认数值不调优，任何回归失败先解释并修复，不改验收阈值制造通过。

## 范围与风险

三缓冲的单写者和生命周期静止条件是接口契约；无锁交换不等于整体 WBC 硬实时。
规划仍是已知矩形平台的有限单步模板。ROS 消息新增字段要求重编相关工作区。
接触仍依赖 effort 残差，不能称为足底传感器测量。动态侧碰/滑移/关节时戳/求解超时
等未覆盖项单列，不把输入级测试或历史 100 次矩阵当成本次全面验收。

实际结果在实施完成后记录于 `refactoring-results.md`，未执行项目明确保留。
