# 香橙派与真机

香橙派必须在目标 ARM64 系统重新构建，不要复制 x86 开发机生成的库或 OCS2 CppAD
缓存：

```bash
cd /path/to/Dog/Dog-control
source /opt/ros/humble/setup.bash
source /path/to/Dog/Dog-control/external/model_ws/install/setup.bash
source /path/to/Dog/Dog-control/external/ocs2_ws/install/setup.bash

export UNITREE_ACTUATOR_SDK_ROOT=/absolute/path/to/unitree_actuator_sdk
colcon build --packages-up-to custom_dog_control \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=ON \
  -DUNITREE_ACTUATOR_SDK_ROOT="$UNITREE_ACTUATOR_SDK_ROOT"
```

只有独立物理急停完成安装和验收后，才允许启动真实硬件：

```bash
source install/setup.bash
ros2 launch custom_dog_control real.launch.py \
  physical_estop_verified:=true \
  calibration_hip_deg:=0.0 \
  calibration_thigh_deg:=71.8 \
  calibration_calf_deg:=-161.8
```

`physical_estop_verified:=true` 只是操作者确认，不会替代硬件急停。物理急停必须独立于
ROS、香橙派和控制进程，能直接撤销执行器使能或动力电源。

真机配置当前保留与仿真相同的最大指令包线，但这只是限幅值，不代表已经通过高速
验收。首轮测试必须限制为低速，并严格按以下顺序推进：

1. 验证物理急停能够独立断开执行器。
2. 单电机检查 ID、方向、减速比、温度和力矩限制。
3. 单腿悬空完成标定、位置插值和安全阻尼测试。
4. 四腿悬空检查 12 电机同步、故障锁存和 WBC 输出方向。
5. 测量完整 RS485 回路的平均、P95、P99 周期和超时率。
6. 使用 `use_sim_ground_truth: false` 验证 IMU、编码器和接触计划估计器。
7. 悬挂完成 WBC 站立，再在安全绳保护下进行落地站立和低速六方向测试。
8. 只有连续测试无超时、温升、限位、求解失败或振荡时，才逐级扩大速度。

当前 1000 Hz 控制周期的名义时间为 `1.0 ms`。在香橙派和完整 12 电机通信链路上，
控制周期 P99 应不超过 `1.2 ms`，并且不能产生新的 IO 超时：

```bash
ros2 run custom_dog_control profile_runtime.py --ros-args \
  -p duration_s:=1800.0 \
  -p control_rate_hz:=1000.0 \
  -p output_csv:=/tmp/custom_dog_control_profile.csv
```

真机估计器使用 IMU、编码器、Pinocchio 运动学和计划接触相位。第一版没有足端传感器，
也没有基于真实触地信号的相位修正，这是仿真迁移到真机时的主要剩余风险之一。

模型与坐标约定见 [软件架构](architecture.md)。
