# 源码注释约定与阅读入口

本轮将机械式逐行说明改为面向维护的工程注释。对上次处理清单中的
2,497 个源码、构建和配置文件进行了基线对比：2,466 个恢复为原始内容，
22 个 C++ 文件和 9 个 Python 文件补充或改写了关键约定。
这表示全范围清理，不表示每个依赖文件都经过逐行人工审阅。

## 注释约定

- 接口说明职责、输入输出、单位、坐标系、数组顺序、有效性和生命周期。
- 实现说明设计原因、数据交接、边界条件及失败后的处理路径。
- 对代码中的隐含假设明确说明，不把尚未实现的检查、同步或容错写成保证。
- 不为 include、普通赋值、括号或返回语句重复添加语法解释，不使用“中文注释”前缀。
- 保留许可证及已有的有效说明；参数默认值以配置和实现为准，不在多处重复抄写。
- 第三方依赖保留原有注释，本项目的单位转换、模型映射和适配约定写在调用边界。

## 控制链路阅读入口

| 文件 | 注释关注点 |
| --- | --- |
| [ControlTypes.hpp](../src/custom_dog_control/include/custom_dog_control/control/ControlTypes.hpp) | 硬件关节顺序、速度/IMU 单位、质心状态布局、接触位掩码 |
| [ModelValidator.hpp](../src/custom_dog_control/include/custom_dog_control/nmpc/ModelValidator.hpp) | 校验结果的使用前提、模型槽位与 joint_id 的区别、模型缓存副作用 |
| [NmpcBackend.hpp](../src/custom_dog_control/include/custom_dog_control/nmpc/NmpcBackend.hpp) | 后台求解与控制线程边界、目标合并、策略时效、WBC 返回约定 |
| [NmpcWbcController.cpp](../src/custom_dog_control/src/controller/NmpcWbcController.cpp) | 周期执行顺序、输入快照、参考速度限制、策略缓存与安全检查 |
| [ControllerStateMachine.cpp](../src/custom_dog_control/src/controller/ControllerStateMachine.cpp) | 起身稳定条件、PD/WBC 交接、步态迟滞及新策略确认、失败回退 |
| [KinematicStateEstimator.hpp](../src/custom_dog_control/include/custom_dog_control/nmpc/KinematicStateEstimator.hpp) | 滤波状态、世界系、规划接触和平地假设、输出有效性 |
| [IOSDK.hpp](../src/custom_dog_control/include/custom_dog_control/hardware/IOSDK.hpp) | 同步收发、每腿工作线程、借用缓冲区、标定前后的位置语义 |
| [UnitreeSystemInterface.cpp](../src/custom_dog_control/src/hardware/UnitreeSystemInterface.cpp) | read/write 的实际发送时序、急停确认条件、通信失败计数 |
| [SafetyMonitor.hpp](../src/custom_dog_control/include/custom_dog_control/safety/SafetyMonitor.hpp) | 故障锁存与显式复位、判定层和命令执行层的职责 |

生命周期、输入仲裁、诊断、混合力矩插值及时间统计的说明位于对应文件中。
模型校验检查的是模型约定，不能替代实物零点标定；规划接触也不是触地测量。

## 训练与部署阅读入口

`himloco_custom_dog/source/himloco_lab/himloco_lab/` 中的说明重点如下：

- `rsl_rl/modules/him_estimator.py`：观测切片、带缩放的速度监督、原型分配与梯度边界。
- `rsl_rl/modules/him_actor_critic.py`：非对称观测、历史帧顺序、采样动作和部署均值。
- `rsl_rl/algorithms/him_ppo.py`、`rsl_rl/storage/him_rollout_storage.py`：旧策略快照、超时处理、GAE 和批次对齐。
- `rsl_rl/runners/him_on_policy_runner.py`、`rsl_rl/wrappers/himloco_vec_env_wrapper.py`：reset 前后观测的用途、历史排列和优化器状态。
- `tasks/locomotion/robots/go2/velocity_env_cfg.py`：45 维单帧观测及 critic 速度标签的位置。
- `utils/export_policy.py`、`utils/export_deploy_cfg.py`：固定维度、潜变量归一化、总历史帧数和部署配置的精度。

这些注释描述当前实现。例如，历史缓冲区未按 dones 清空、JIT 导出固定单帧
45 维等行为被明确记录，本轮不顺带修改训练或部署逻辑。

## 验证

- 2,466 个恢复文件与上次注释前的备份逐字节一致，包含已处理的第三方和外部依赖文件。
- 22 个改写 C++ 文件的有效 token 及有效代码行结构与基线一致。
- 9 个改写 Python 文件的 AST 与基线一致，并通过编译语法检查；文档字符串也未改变。
- 两个主仓库的 `git diff --check` 通过。
- `legged_control_upstream.sha256` 恢复原清单，未以改写来源哈希来掩盖依赖变化。

- `custom_dog_control` 现有构建配置编译成功；该配置关闭真机硬件目标，
  硬件文件本轮通过的是有效代码等价检查，不代表完成了真机编译或联调。
- CTest 6/6 通过：第三方来源、控制约定、混合命令工具、安全监控、URDF 约定、模型验证。

随后已完成 Gazebo 平地基础回归，实测指标和复现命令见
[2026-09-29 仿真验证记录](simulation-validation-20260929.md)。
未启动真机或 Isaac Lab 训练，未运行所有外部依赖的完整测试套件。
