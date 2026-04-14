# qr_guide

`qr_guide` 是当前这台四足机器人的 ROS2 主控包。现在仓库已经清理成一条主线，只保留真实会编译、会运行、会维护的代码，不再混着旧状态机、旧步态和旧控制器。

## 1. 对外接口

- ROS2 package: `qr_guide`
- executable: `junior_ctrl`
- launch: `launch/dog.launch.py`
- input topics: `/imu`、`/joy`

## 2. 当前目录结构

当前主线只保留这些模块：

- `config/`
  - 机器人参数唯一来源
- `model/`
  - 单腿运动学和整机模型
- `input/`
  - 手柄映射
- `runtime/`
  - ROS2 输入采集和控制循环调度
- `interface/`
  - 真实电机串口通信、校准和回读
- `control/`
  - 状态估计器
- `FSM/`
  - 主状态机，只保留 4 个主状态

当前仍在主线中的核心文件：

- `src/main.cpp`
- `src/config/RobotConfig.cpp`
- `src/model/LegKinematics.cpp`
- `src/model/RobotModel.cpp`
- `src/input/JoystickMapper.cpp`
- `src/runtime/ControllerNode.cpp`
- `src/runtime/RobotRunner.cpp`
- `src/runtime/VisualizationPublisher.cpp`
- `urdf/custom_quadruped.urdf.template`
- `src/interface/IOSDK.cpp`
- `src/control/Estimator.cpp`
- `src/FSM/FSM.cpp`
- `src/FSM/State_Passive.cpp`
- `src/FSM/State_FixedStand.cpp`
- `src/FSM/State_Trotting.cpp`
- `src/FSM/State_StepTest.cpp`

## 3. 统一坐标系

当前主线统一采用这一套坐标系约定：

- `body / hip frame`: `x` 向前，`y` 向机器人右侧，`z` 向上
- 因此 `FR / RR` 的 `y` 必须为正
- `FL / RL` 的 `y` 必须为负

这套规则同时约束：

- `hip_mounts_in_body`
- `stand_targets.normal_feet_in_hip`
- `stand_targets.crouch_feet_in_hip`

参数加载时会做一致性检查，如果左右腿的 `y` 符号写反，程序会直接报错退出，不再静默带错运行。

说明：

- 上面这套是控制器内部坐标系，运动学、校准和状态机都按它工作
- RViz 可视化层会额外对位置、腿部几何和轨迹做一次左右反射，把显示转换成更符合 ROS/RViz 习惯的右手系
- IMU 姿态本身保持原始 ROS 方向，不做偏航镜像
- 因此真机控制语义不变，但 RViz 里左右腿不会再镜像反掉

## 4. 参数文件

当前主线参数统一放在：

- `config/custom_quadruped.yaml`

主程序启动时从安装目录读取该文件，不再依赖源码目录：

- `share/qr_guide/config/custom_quadruped.yaml`

当前锁定的关键参数：

- 总质量：`13.75 kg`
- 机身质量：`6.55 kg`
- 单腿质量：`1.80 kg`
- 大腿质量：`1.55 kg`
- 小腿质量：`0.25 kg`
- 机身尺寸：`[0.250, 0.223, 0.110] m`
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
  - `FR = /dev/ttyS3`
  - `FL = /dev/ttyS4`
  - `RR = /dev/ttyS7`
  - `RL = /dev/ttyS8`
- 腿部串口并发：
  - `use_parallel_leg_io = false`
- 减速比：
  - `base_gear_ratio = 6.33`
  - `calf_total_gear_ratio = 12.66`
- 关节限位：
  - `q0 = [-2.60, 2.60]`
  - `q1 = [-6.50, 6.50]`
  - `q2 = [-2.30, 2.30]`

## 5. 控制链路

当前控制循环顺序是：

1. `main.cpp` 加载参数并创建运行对象
2. `ControllerNode` 订阅 `/imu` 和 `/joy`
3. `RobotRunner` 每周期读取一次输入快照
4. `JoystickMapper` 把手柄映射成 `userCmd / userValue`
5. `IOSDK` 完成电机收发、减速比换算、校准偏差和回读
6. `Estimator` 根据 IMU、关节状态和接触信息估计机体状态
7. `FSM` 根据当前状态输出关节命令
8. `VisualizationPublisher` 把估计结果、TF、关节状态和足端轨迹发布到 RViz

说明：

- 当前估计器仍是线性卡尔曼滤波框架
- `_feetH` 当前不再固定写死为 `0`
- 而是按 `leg_geometry.foot_radius_m` 作为平地参考高度
- 当前配置中取 `0.025 m`
- 对应假设是“世界系原点在地面，足端建模点是脚端球心/脚垫中心”

## 6. 主状态机

主构建当前只保留 4 个状态：

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

- `L1_X` 只作为校准触发，不承担切状态职责
- 旧的 `FreeStand / BalanceTest / SwingTest / move_base` 已经从仓库中删除

## 7. 运动学

当前运动学已经统一封装到：

- `src/model/LegKinematics.cpp`
- `src/model/RobotModel.cpp`

这套实现保持的是你原来那套关节定义：

- 前腿对 `q0` 做镜像
- 左腿对 `q1 / q2` 做镜像
- 小腿绝对角使用 `q2_abs = pi/2 - q2`

当前 `FK / IK / Jacobian` 都走这一套定义，不再分散在状态文件和旧模型里。

## 8. 校准与电机链路

`IOSDK` 当前只负责真实硬件链路：

- 串口打开
- 电机命令发送
- 电机状态回读
- 减速比换算
- 校准偏差 `_calibOffset`
- 默认采用 `FR -> FL -> RR -> RL` 单线程顺序收发，避免 4 腿并发串口带来的时序抖动

校准触发逻辑：

1. 手动把机器人摆到 L 型校准姿态
2. 按 `START`
3. 读取当前电机回传角
4. 用 `当前实测角 - 理想零位角` 计算 `_calibOffset`
5. 后续发命令时加偏差，回读时减偏差

当前校准目标角：

- 髋关节：`0.0 deg`
- 大腿：`-161.8 deg`
- 小腿：`-71.8 deg`

对应公式：

- 命令路径：`q_motor = q_user + calib_offset`
- 回读路径：`q_user = q_feedback - calib_offset`

## 9. 校准后的调试打印

按下 `START` 校准完成后，主程序会进入一个调试打印模式：

- 先强制打印一帧当前四条腿的姿态
- 后续只有当关节角或足端坐标明显变化时才继续打印

打印内容包括：

- `q(rad)=[q0, q1, q2]`
- `foot_hip(m)=[x, y, z]`
- `foot_body(m)=[x, y, z]`

同时，状态估计也会进入动态调试打印：

- `position(m)=[x, y, z]`
- `velocity(m/s)=[vx, vy, vz]`
- `contact=[FR, FL, RR, RL]`
- `phase=[FR, FL, RR, RL]`
- `foot_h_ref(m)=[FR, FL, RR, RL]`

这个输出主要用于你手动摆腿时核对：

- 当前回读关节角是否合理
- FK 算出来的足端位置是否符合机械直觉
- body/hip 坐标系的 `y` 方向是否定义一致
- 状态估计的位置、速度和接触相位是否符合当前动作
- `_feetH` 的参考高度是否和脚端半径约定一致

## 10. 构建与运行

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

## 11. RViz 可视化

当前已经内置一套完整的 RViz 可视化链路，启动后可以直接看到：

- `robot_state_publisher` 驱动的完整狗模型
- `odom -> base_link_est` TF
- 机身状态估计 `odom`
- 机身历史轨迹 `path`
- 四个实际足端轨迹
- 四个命令足端轨迹
- 机身速度箭头
- 当前状态、接触相位和估计值文字面板

当前模型外观说明：

- 机身直接使用参数驱动的圆角盒体，尺寸仍严格跟随 `body_size_m`，但边缘会更圆滑
- 腿部不再直接套 Go2 的原始 mesh，而是按你自己的 `hip_mount / L0 / L1 / L2 / foot_radius` 直接生成圆润的参数化模型
- 髋电机、大腿电机、膝关节、小腿和足端都严格挂在同一条关节链上，因此可视化长度和连接关系与真实参数一致
- 整体外观只学习 Go2 的视觉语言，例如深浅双色机身、电机壳层次、腿部包覆外壳和脚垫风格，但几何尺寸优先服从你自己的机器人参数
- 当前外观进一步简化为：机身白色圆角盒体、连杆白色圆柱、脚垫白色球体，关节统一黑色扁平圆柱，方便直接观察关节链是否对齐
- marker 层的腿骨架、足端点和轨迹也同步做了压细处理，叠加 URDF 时更清爽

这条链路的组成是：

- `launch/dog.launch.py`
  - 读取 `config/custom_quadruped.yaml`
  - 用 `urdf/custom_quadruped.urdf.template` 生成当前机器人的 `robot_description`
  - 直接按机器人参数生成长方体机身和圆润的参数化腿部外观
  - 启动 `robot_state_publisher`
- `VisualizationPublisher`
  - 发布 `/joint_states`
  - 发布 `/tf` 中的 `odom -> base_link_est`
  - 发布估计 `odom/path`
  - 发布足端和状态 marker

说明：

- URDF 只用于 RViz 和 TF，不参与控制求解
- 关节角来源是实时电机回读，不是命令角
- URDF 关节轴、左右镜像和前后腿髋关节符号，已经和当前 `LegKinematics` / `IOSDK` 语义对齐
- 控制器内部仍然是 `y` 向机器人右侧；RViz 发布层会把点和 URDF 几何统一反射到显示坐标系，但 IMU 姿态保持原始方向

主要 topic：

- `/qr_guide/estimation/odom`
- `/qr_guide/estimation/path`
- `/qr_guide/visualization/marker_array`
- `/joint_states`
- `/tf`
- `/robot_description`

启动命令：

```bash
cd /home/orangepi/qr_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qr_guide dog.launch.py
```

可视化口径：

- 机身位置来自状态估计 `Estimator::getPosition()`
- 机身姿态来自 IMU 四元数
- 实际足端来自当前回读关节角 FK
- 命令足端来自当前下发关节命令 FK

默认 launch 已经支持一并启动 RViz：

```bash
ros2 launch qr_guide dog.launch.py
```

如果不想启动 RViz：

```bash
ros2 launch qr_guide dog.launch.py use_rviz:=false
```

默认 RViz 配置文件位于：

- `share/qr_guide/rviz/qr_guide_visualization.rviz`

运行前建议按这个顺序检查：

1. `/imu` 是否正常
2. `/joy` 是否正常
3. 4 个串口是否和腿序 `FR / FL / RR / RL` 一一对应
4. 先离地完成校准
5. 先验证 `PASSIVE -> FIXED_STAND`
6. 再验证 `FIXED_STAND -> TROTTING`
7. 最后验证 `STEP_TEST`

## 11. 清理状态

当前仓库已经删除不再参与主线构建的遗留模块，代码树保持为“只剩主线”的状态。后续如果继续维护，建议继续遵守下面两条：

- 任何新参数只进 `config/custom_quadruped.yaml`
- 任何新坐标约定都必须和当前 `x forward / y right / z up` 保持一致
