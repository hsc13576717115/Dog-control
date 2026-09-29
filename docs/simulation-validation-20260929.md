# 工程注释改写后的仿真验证：2026-09-29

本次验证 `refactor/control-workspace-maintainability` 分支上的工程注释改写。
22 个 C++ 文件的有效代码与基线 `684d500` 一致，第三方依赖与来源哈希清单保持原样。
之前的结构重构、依赖迁移验证见 [2026-09-23 记录](simulation-validation-20260923.md)。

## 环境与执行方式

- ROS 2 Humble、Gazebo Classic 11，关闭 GUI、RViz 和键盘输入。
- 使用项目默认平地、点足模型及 `use_sim_ground_truth: true`。
- 使用独立 `ROS_DOMAIN_ID=109`、`ROS_LOCALHOST_ONLY=1` 和动态 Gazebo master 端口。
- 每个场景重新启动 Gazebo，场景结束后由现有回归运行器清理其进程组。
- 通过官方构建入口更新模型 underlay 和安装目录后再测试，未修改控制参数或验收阈值。

## 结果

| 检查 | 结果 |
| --- | --- |
| 模型 underlay 与主工作区构建 | 成功，主工作区 3 个包完成 |
| 单元与契约测试 | 6 个 CTest 入口；colcon 汇总 23 tests，0 errors / failures / skipped |
| 运动链 | 起身 → MPC_STANCE → MPC_TROT → MPC_STANCE，通过 |
| 零速度使能 Trot | 保持全支撑，位移未超过测试的 0.03 m 上限 |
| 12 秒前进，0.05 m/s | dx +0.575 m，dy -0.001 m，偏航漂移 +0.002 rad |
| 上述运动最大 roll/pitch | 0.005 / 0.010 rad，停车后 x/y 速度约 0 m/s |
| 矩阵前进，0.05 m/s、8 秒 | dx +0.380 m，dy -0.004 m |
| 矩阵侧移，0.04 m/s、8 秒 | dx -0.011 m，dy +0.316 m |
| 矩阵转向，0.18 rad/s、8 秒 | yaw +1.440 rad，dx -0.008 m，dy -0.005 m |
| 矩阵 WBC 与停车 | 三段 invalid_wbc=0，每段停车后均保持 MPC_STANCE |
| 独立 smoke 冷启动 | 20 秒起身/站立观察通过；末帧高度约 0.286 m，roll/pitch 约 0.000/-0.002 rad |

三组场景的启动日志未发现控制器 FAULT、求解器停止或策略评估异常。
日志中的 Q/R 默认值提示以及构建中的第三方兼容性/弃用警告不影响本次通过结果。
这些指标是本机单次运行结果，不是实时性或全工作包线保证。

## 复现与日志

```bash
CUSTOM_DOG_BUILD_ONLY=1 src/custom_dog_control/scripts/build_simulation.sh
source src/custom_dog_control/scripts/env.sh
colcon test --packages-select custom_dog_control --event-handlers console_direct+
colcon test-result --test-result-base build/custom_dog_control --verbose
src/custom_dog_control/scripts/test_simulation.sh \
  --scenario all --domain-id 109 \
  --output-dir log/my-engineering-comments-validation
```

输出目录必须尚不存在，ROS 域需确认未被其他测试使用。本次实际日志保存在
`log/engineering-comments-validation-20260929/`，包含构建日志、单元测试日志，
以及 `basic/motion/`、`basic/matrix/`、`standing/smoke/` 下的启动日志、测试输出和结果 JSON。
日志目录按仓库规则不提交 Git。

smoke 场景使用现有运行器的相同隔离/清理流程启动 `simulation_smoke_test.py`。
手动复现时，在全新 Gazebo 的同一 ROS 域运行
`ros2 run custom_dog_control simulation_smoke_test.py`。

本次未重跑高速包线、地形矩阵、关闭真值后的状态估计器闭环或真机测试。
相邻训练仓库的 9 个 Python 注释改动通过 AST 等价及语法检查；
当前 Python 环境不含 torch、isaaclab、onnxruntime，因此未运行训练或 ONNX 推理。
