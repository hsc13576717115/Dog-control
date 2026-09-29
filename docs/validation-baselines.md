# 仿真验收与历史基线

以下高速包线与地形数据保留自仓库原 README，未作为本次工程化重构的重新验收结果。
本次验证范围和实测结果见 [本机验收记录](simulation-validation-20260923.md)。
每项测试应使用独立冷启动；基础 motion/matrix 可用根 README 中的自动入口。

## 仿真验收

自动测试时关闭 GUI、RViz 和键盘节点：

```bash
ros2 launch custom_dog_control gazebo.launch.py \
  gui:=false use_rviz:=false start_keyboard:=false
```

在另一个已 source 相同环境的终端运行：

```bash
ros2 run custom_dog_control simulation_smoke_test.py
ros2 run custom_dog_control simulation_motion_test.py
ros2 run custom_dog_control simulation_command_matrix_test.py

ros2 run custom_dog_control simulation_envelope_test.py --vx 1.5
ros2 run custom_dog_control simulation_envelope_test.py --vx -1.5
ros2 run custom_dog_control simulation_envelope_test.py --vy 1.0
ros2 run custom_dog_control simulation_envelope_test.py --vy -1.0
ros2 run custom_dog_control simulation_envelope_test.py --yaw 2.0
ros2 run custom_dog_control simulation_envelope_test.py --yaw -2.0
```

每项速度包线都必须使用全新 Gazebo 世界，依次完成起身、20 秒满量程运动、受控停车
和 10 秒 MPC_STANCE 保持。当前固定 `0.25 s` Trot 周期的独立冷启动结果为：

| 目标 | 稳态实测 | roll / pitch RMS | 最大高度误差 |
| --- | --- | --- | --- |
| `vx +1.5 m/s` | `+1.472 m/s` | `0.005 / 0.008 rad` | `0.026 m` |
| `vx -1.5 m/s` | `-1.475 m/s` | `0.003 / 0.009 rad` | `0.025 m` |
| `vy +1.0 m/s` | `+1.000 m/s` | `0.018 / 0.008 rad` | `0.016 m` |
| `vy -1.0 m/s` | `-0.994 m/s` | `0.019 / 0.007 rad` | `0.016 m` |
| `yaw +2.0 rad/s` | `+2.008 rad/s` | `0.002 / 0.004 rad` | `0.008 m` |
| `yaw -2.0 rad/s` | `-2.007 rad/s` | `0.002 / 0.003 rad` | `0.008 m` |

线速度容差为 `0.20 m/s`，角速度容差为 `0.30 rad/s`。这些数据是在
`use_sim_ground_truth: true` 下得到的，证明当前控制与接触模型在 Gazebo 中可运行，
不能代替真机估计器和硬件实时性验收。

### 地形通过测试

`world` Launch 参数可选择独立地形世界。每项测试都从原点冷启动，不要自行改变出生
坐标把机器人直接放到障碍附近，因为当前 NMPC 参考初始化假定原点出生。
以 5 cm 台阶为例：

```bash
# 终端 1
WORLD_DIR="$(ros2 pkg prefix --share custom_dog_control)/worlds"
ros2 launch custom_dog_control gazebo.launch.py \
  gui:=false use_rviz:=false start_keyboard:=false \
  world:="$WORLD_DIR/step_50mm.world"

# 终端 2
ros2 run custom_dog_control simulation_terrain_test.py \
  --name step_50mm --speed 0.25 --target-distance 3.2
```

测试节点要求起身交接后连续稳定站立 1 秒，再以 `0.25 m/s` 前进；通过条件为完成目标
距离、NMPC/WBC 全程有效、roll/pitch 不超过 `0.45 rad`、横漂不超过 `0.35 m`、
基座高度不低于 `0.12 m`，并能停车恢复 MPC_STANCE。独立冷启动结果如下：

| 世界 | 完成距离 | 最大 roll / pitch | 最大横漂 | 基座高度范围 | 结果 |
| --- | --- | --- | --- | --- | --- |
| `step_30mm.world` | `3.200 / 3.2 m` | `0.055 / 0.065 rad` | `0.013 m` | `0.281-0.315 m` | 通过 |
| `step_50mm.world` | `3.200 / 3.2 m` | `0.087 / 0.113 rad` | `0.025 m` | `0.279-0.355 m` | 通过 |
| `ramp_5deg.world` | `4.200 / 4.2 m` | `0.011 / 0.026 rad` | `0.010 m` | `0.283-0.324 m` | 通过 |
| `ramp_10deg.world` | `2.485 / 4.2 m` | `0.071 / 0.037 rad` | `0.040 m` | `0.282-0.415 m` | 失败：坡顶停滞 |
| `uneven_10_40mm.world` | `4.001 / 4.0 m` | `0.058 / 0.057 rad` | `0.013 m` | `0.282-0.312 m` | 通过 |
| `low_friction_mu_020.world` | `4.200 / 4.2 m` | `0.009 / 0.010 rad` | `0.011 m` | `0.282-0.289 m` | 通过 |

当前结果表示平地控制器具有一定被动越障余量，不表示已经实现地形自适应。现有
`SwitchedModelReferenceManager` 仍把摆动足地面高度固定为 `0.0 m`，没有高程图、
落脚点重规划或真实触地修正。10° 坡顶停滞与该限制一致；继续提高坡度、台阶高度或
速度前，应先把地形高度和法向量接入 NMPC 参考、摆动足轨迹与 WBC 接触任务。
`mu=0.20` 结果仅验证 `0.25 m/s` 直行通过，不构成完整的低摩擦速度包线。
