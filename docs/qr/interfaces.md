# QR 接口与执行边界

[中文入口](README.md) | [English overview](README_EN.md)

## 坐标与时间

控制帧为连续 `odom`，机身帧为 `base`，足序 FR/FL/RR/RL。ROS 四元数 XYZW，内部 IMU WXYZ；边界显式转换。未来全局定位只修正 `map→odom`，不能移动正在承载的足端锚点。

所有有效性判断必须保留采样时刻。当前精确实现仅允许 Gazebo，尚未完成每腿真实硬件采样时间和时钟同步接入。`pose_covariance` 对角线为 -1 表示当前接口不提供协方差，不代表零不确定度。

## 输入输出

| 接口 | 含义 |
| --- | --- |
| `/imu`、ros2_control 关节状态 | 控制唯一传感输入；关节必须有有效 q/dq/effort |
| `/qr/support_regions` | `SupportRegionArray`；当前由已知场景源发布，不是感知结果 |
| `/qr/plan_footsteps` | `PlanFootsteps`；足序、足心目标与支撑面 ID，返回计划或失败原因 |
| `/qr/preview_footsteps` | 1–16 步的只读后续可行性预检；不分配计划 ID，不授权执行 |
| `/qr/execute_footsteps` | `ExecuteFootsteps` action；非实时校验后通过缓冲交给 update |
| `/qr/state` | `RobotState`；控制估计、关节、FK 足心和接触估计，无基座真值 |
| `/qr/contact_estimates` | `FootContactArray`；force_valid=false，estimated_force 单独标记为动力学残差估计 |
| `/qr/execution` | 阶段、计划编号、目标、失败原因和求解质量 |
| `/evaluation/link_states`、`/evaluation/contact_*` | 仅供独立评价，不输入控制器 |

目标为 **足心位置**，不是地面接触点。当前模型足半径 0.026 m，水平面目标足心 z=表面高度+0.026。目标 x/y 必须位于按足半径与 20 mm 误差裕量收缩后的凸区域内。新模型必须重新核对足底尺寸。

## 线程与时间边界

`PrecisionChannel` 的控制状态使用固定三缓冲：单个控制线程每次一次原子交换发布，
非实时读者串行取得完整副本，不返回可变指针。未读到的中间状态可以被后续状态覆盖。
快照仅含固定字段，错误为枚举；重置要求 update 停止。规划指令反向通过固定大小
`PrecisionCommand` 的 RealtimeBuffer 进入 update，不在控制周期写入非实时缓冲。

控制快照新鲜度按采集/计算时记录的 `steady_stamp` 检查，ROS/仿真时间用于消息头、
地图观测与计划有效期。不能用刚收到快照的时刻覆盖其采集时刻。Gazebo 的控制 update
与节点 `/clock` 交付可能相差一个周期，不能把这种差异误判成非法未来状态。
IMU 的原始采样时刻仍单独检查，重复投递旧 IMU 不能刷新有效期。

无锁状态交换不代表整个 WBC 回路无动态内存或具有硬实时保证。求解器分配、调用耗时
及 NX 的实际时延仍需后续验证。

## 协议与配置

`ExecutionStatus.protocol_version=1`；新增 `phase_code` 和 `error_code`，原 `phase/error`
文字保留。阶段码固定为 0站立、1重心转移、2摆动、3触地确认、4恢复、5保持、6完成。
服务和 action 结果新增 `error_code`，数值表见 `qr_interfaces/msg/ExecutionError.msg`；
成功为 0，旧编号不得复用。ROS action 准入拒绝发生在 goal 接受前，ROS 标准响应只提供
accepted=false，不伪造不存在的 result。消息类型变化后必须重新构建上下游。

- 模型：`custom_dog_control/config/precision_model.yaml`，足半径须与 URDF 碰撞球匹配。
- 控制/规划：`precision_control.yaml`，接触阈值、裕量、时序、WBC 增益、求解预算。
- 验收：`qr_validation/config/acceptance.yaml`，只由评价器读取，不影响控制输出。
- 质量、惯量、关节及力矩限位仍来自 URDF；WBC 摩擦/任务约束保留 `config/nmpc/task.info`。

配置仅在初始化时加载，不热更新。缺字段、未知字段、非有限值、越界和时序/迟滞冲突
导致配置失败；不静默回退到默认值。`.def` 文件维护 C++ 默认值与允许范围，YAML 是实际
启动输入。数值积分容差、算法维数和迭代上限仍是实现常量，不宣称所有数字都可调。

## M1 执行

PRECISION_STANCE → SHIFT → SWING → CONTACT_CONFIRM → RESTORE → DONE。

无有效支撑、侧碰、超时或取消进入 CONTROLLED_HOLD，并返回失败。接触估计只有载荷与运动学证据持续成立后才确认；单纯摆腿时间结束不能完成一步。严重状态/WBC 故障锁存并终止执行，当前仿真用阻尼安全输出；尚未证明真实高处支撑故障能够安全恢复。

落地阶段先施加有界向下预载（最大 6 N 等效力，最终关节力矩仍受限），
以获得可观测的 effort 残差；持续小载荷允许尝试转移负载，达到更高载荷且持续确认后才完成。
目标下探超过 8 mm 或超时即中止。预载命令本身不是触地证据。
接触不可信、滑移或摆腿时移除对应估计约束；支撑锚点在确认时由估计 FK 捕获，
不使用 Gazebo 接触位置或全局定位修正更新它。

M1 单步服务包含轨迹采样 IK、关节边界、已知矩形平台与地面碰撞及 WBC 可行性检查。已通过计划有有限接收有效期；执行动作不得改变预检内容。公共消息中的多步数组为后续接口预留，当前长度必须为 1。

规则计分、任务覆盖和 StartMission 均属于后续阶段；消息类型存在不代表模块已实现。


## 当前实现的频率与范围

- 仿真控制配置 1000 Hz，IMU 250 Hz；估计接触依赖关节 effort，不能将重复读取 IMU 算作新采样。
- 状态、接触估计、执行反馈发布目标 50 Hz；已知支撑面发布 10 Hz，规划为服务触发。
- 单步规划以实际模型质心为依据寻找支撑三角形内的 30 mm 裕量位置，
  尝试 0/15/30 mm 机身抬升候选，采样检查 IK、碰撞及 WBC 可行性。
  这是有限局部模板，不是通用接触搜索；高墙、沟壑和梅花桩尚未接入。
- 当前 WBC 求解预算为 20 ms 保护上限，不是承诺每个 1 ms 周期都准时。
  求解残差/耗时记录在执行状态，NX 和高负载实时性未验收。
- 碰撞预检使用原始模型的完整腿部/机身碰撞体，对 M1 已知轴对齐矩形平台和地面检查。
  增加了规范 CAD 非相邻连杆的离散自碰撞预检；尚未加入网格地图、多层净空、连续碰撞检测或任意支撑法向。
- 全局定位、足端锚点重定位、真实每电机采样时刻、时钟同步与 effort 标定属于后续工作。

## M2 连续换步扩展

- `Footstep.body_target` 是卸载前移重心目标；`body_finish` 是确认承载后的机身结束目标。二者均为 `odom` 下米单位机身位置参考，不能由 action 客户端改写已批准内容。
- `precision_m2.yaml` 显式启用 `body_follow_ratio=0.25`：每次将机身终点推进该足位移的四分之一；默认 M1 配置仍为 0。它只覆盖有限、缓慢、水平已知支撑面的换步，不支持任意机身姿态轨迹。
- 整条模板先经过 `PreviewFootsteps`；内部预测状态不发布、不进入估计器。随后每步重新规划、校验计划新鲜度和地图版本，等待实际估计承载后才进入下一步。预检通过不是执行保证。
- `PlanFootsteps` 和 `PreviewFootsteps` 在专用非实时单线程 executor 上串行运行，避免 CAD/多步预检饿死 IMU 回调。计算期间不持有发布/action 的互斥锁；批准前重新核对活动状态、忙碌状态和地图版本。模型加载/计算异常返回固定错误码 `PLANNING_EXCEPTION=36`，不会授权关节命令。
- M2 的球足模式使用 `v_contact = v_center + omega × (-r*n)`；在已确认水平支撑上积分球心滚动，WBC 与残差估计使用接触点雅可比。没有新增足底传感器；滑移仍是模型估计，实际接触与滑移只由独立仿真评价器判定。
- 场景碰撞仍使用完整规范碰撞体。非相邻自碰撞使用同一 URDF 原始 CAD 网格，因为规范简化机身/大腿碰撞体在站姿存在互相重叠；只排除同刚体和直接连接的连杆。此检查不等于 Gazebo 实际启用了机器人所有自碰撞，也不等于连续碰撞认证。
- ROS 消息增加字段后，上下游必须一起重建。动作仍只接受一个已批准步骤；离线 `precision_envelope` 输出的协同位姿采样不能直接发送给当前 action。
