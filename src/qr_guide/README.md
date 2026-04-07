# qr_guide

这是 `qr_ws/src/qr_guide` 的主控包，当前版本已经按“参数驱动 + 运行器 + FSM + 硬件接口”的思路做过一轮重构，目标是把原来分散在多个文件里的硬编码参数、运动学、手柄映射和控制入口统一起来，同时尽量保持原有外部接口不变。

当前主线统一采用这一套坐标系约定：

- `body / hip frame`: `x` 向前，`y` 向机器人右侧，`z` 向上
- 因此 `FR / RR` 的 `y` 为正，`FL / RL` 的 `y` 为负
- `hip_mounts_in_body` 和 `stand_targets.*_feet_in_hip` 都必须遵守这套约定

## 1. 对外兼容内容

- ROS2 包名仍然是 `qr_guide`
- 可执行程序仍然是 `junior_ctrl`
- launch 入口仍然是 `dog.launch.py`
- 输入话题仍然使用 `/imu` 和 `/joy`

## 2. 当前主线架构

当前主线只编译和维护这些模块：

- `config/`
  - 机器人参数的唯一来源
- `model/`
  - 腿部运动学和整机模型
- `interface/`
  - 真实电机串口通信与校准
- `runtime/`
  - ROS2 输入采集与控制循环调度
- `FSM/`
  - 主状态机

控制链路如下：

1. `main.cpp` 加载参数文件并创建运行对象
2. `ControllerNode` 订阅 `/imu` 和 `/joy`
3. `RobotRunner` 在固定周期内完成一次控制步
4. `JoystickMapper` 把手柄输入映射成 `userCmd/userValue`
5. `IOSDK` 完成电机收发与校准偏差处理
6. `Estimator` 估计机体状态
7. `FSM` 根据当前状态输出关节命令

## 3. 主线状态

主构建目前只保留下面 4 个状态：

- `PASSIVE`
- `FIXED_STAND`
- `TROTTING`
- `STEP_TEST`

状态切换逻辑：

- `PASSIVE -> FIXED_STAND`：`L2_A`
- `FIXED_STAND -> PASSIVE`：`L2_B`
- `FIXED_STAND -> TROTTING`：`L2_X`
- `TROTTING -> PASSIVE`：`L2_B`
- `TROTTING -> FIXED_STAND`：`L2_A`
- `TROTTING -> STEP_TEST`：`L2_Y`
- `STEP_TEST -> TROTTING`：动作完成或再次收到 `L2_X`
- `STEP_TEST -> FIXED_STAND / PASSIVE`：保持原有行为

说明：

- `L1_X` 现在只作为校准触发，不再承担切状态职责
- `FreeStand / BalanceTest / SwingTest / move_base` 仍然保留源码，但不参与主构建

## 4. 参数文件

当前主线参数统一放在：

- `config/custom_quadruped.yaml`

已经锁定的关键参数如下：

- 总质量：`13.75 kg`
- 机身质量：`6.55 kg`
- 单腿质量：`1.80 kg`
- 大腿质量：`1.55 kg`
- 小腿质量：`0.25 kg`
- 机身尺寸：`0.250 x 0.223 x 0.110 m`
- 质心偏移：`[0, 0, 0]`
- 机身惯量：`[0.033748, 0.040719, 0.061258]`
- 整机惯量：`[0.284357, 0.486763, 0.358663]`
- 髋安装点：
  - `FR = [0.185,  0.055, 0.0]`
  - `FL = [0.185, -0.055, 0.0]`
  - `RR = [-0.185,  0.055, 0.0]`
  - `RL = [-0.185, -0.055, 0.0]`
- 腿长：
  - `L0 = 0.08415`
  - `L1 = 0.213`
  - `L2 = 0.213`
- 串口：
  - `FR=/dev/ttyS3`
  - `FL=/dev/ttyS4`
  - `RR=/dev/ttyS7`
  - `RL=/dev/ttyS8`
- 减速比：
  - `base_gear_ratio = 6.33`
  - `calf_total_gear_ratio = 12.66`
- 关节限位：
  - `q0 = [-2.60, 2.60]`
  - `q1 = [-6.50, 6.50]`
  - `q2 = [-2.30, 2.30]`

## 5. 运动学逻辑有没有改

结论先说：

- 前向运动学公式按你原来 `State_SwingTest` 里的 `fkCheckAxis` 对齐
- 逆运动学公式按你原来 `State_SwingTest` 里的 `ikCheckAxis` 对齐
- Jacobian 也按这套关节定义重新整理
- 只是把原来散落在 `SwingTest / unitreeRobot / FSM` 里的硬编码参数，统一收口到了参数文件

新实现位置：

- `src/model/LegKinematics.cpp`
- `src/model/RobotModel.cpp`

原始参考实现位置：

- `src/FSM/State_SwingTest.cpp`

这两套实现的核心 FK / IK / Jacobian 公式是一致的，差别主要是：

1. 新版把 `L0/L1/L2` 和 `hip_mount` 从参数文件读取，不再写死在类里
2. 新版把腿序统一成 `FR, FL, RR, RL`
3. 新版通过 `FrameType::HIP / BODY` 明确区分坐标系，并统一成 `y` 向机器人右侧

所以这次更像是“把你原来 SwingTest 的运动学做成统一模型层”，不是换了一套新的推导。

## 6. 机械限位逻辑有没有改

机械限位的数值没有改，只是统一到了参数文件里：

- `q0 = [-2.60, 2.60]`
- `q1 = [-6.50, 6.50]`
- `q2 = [-2.30, 2.30]`

主线状态里会在 IK 结果出来之后再做一次限幅，避免瞬时目标角越界：

- `State_FixedStand`
- `State_Trotting`
- `State_StepTest`

## 7. 初始位置和校准偏差逻辑有没有改

### 7.1 站立和下蹲目标

这部分“目标值来源”改了，但“控制含义”没改：

- 以前：站立脚端目标散落在状态文件里硬编码
- 现在：统一在 `stand_targets` 配置项里管理

这样做的好处是后续改站姿时不需要再同时改多个 `.cpp` 文件。

### 7.2 校准偏差

校准偏差的处理逻辑本质没有改，仍然是：

1. 人工把机器人摆到 L 型校准姿态
2. 按 `START` 触发校准
3. 读取当前电机回传角
4. 用“当前实测角 - 理想 L 型目标角”得到 `_calibOffset`
5. 发命令时加偏差，回读时减偏差

当前仍然使用的 L 型目标角：

- 髋关节：`0.0 deg`
- 大腿：`-161.8 deg`
- 小腿：`-71.8 deg`
- 左右腿的大腿和小腿仍通过 `sign` 处理方向

也就是说：

- 命令路径：`q_motor = q_user + calib_offset`
- 回读路径：`q_user = q_feedback - calib_offset`

变化只有一个：

- 以前是 `IOSDK` 自己订阅 `/joy` 并决定何时校准
- 现在是 `ControllerNode + JoystickMapper` 先把 `L1_X` 写进 `LowlevelState::userCmd`，再由 `IOSDK` 执行同样的校准逻辑

所以“谁触发”这件事的入口更清晰了，但“怎么计算偏差”没有改。

## 8. 构建方式

在工作空间根目录执行：

```bash
cd /home/orangepi/qr_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select qr_guide
```

编译通过后主程序位于：

```bash
install/qr_guide/lib/qr_guide/junior_ctrl
```

## 9. 运行前建议

真机运行前建议按这个顺序检查：

1. `/imu` 话题是否正常
2. `/joy` 话题是否正常
3. 4 个串口是否对应正确腿序 `FR / FL / RR / RL`
4. 先在离地或保护状态下完成校准
5. 先验证 `PASSIVE -> FIXED_STAND`
6. 再验证 `FIXED_STAND -> TROTTING`
7. 最后验证 `STEP_TEST`

## 10. 遗留模块说明

下面这些源码目前保留在仓库中，只作为历史参考，不参与主构建：

- `State_FreeStand`
- `State_BalanceTest`
- `State_SwingTest`
- `State_move_base`
- `BalanceCtrl`
- `GaitGenerator`
- `FeetEndCal`

后续如果继续清理，可以把这部分移动到单独的 `legacy/` 目录中。
