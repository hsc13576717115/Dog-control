# 本机仿真验证：2026-09-23

工作目录：`/home/hsc/Desktop/Dog/Dog-control`。基于 `main/f3997bb`，包含本次启动修复。

## 环境与修复

- ROS 2 Humble、Gazebo Classic 11；OCS2 固定版本的 16 个依赖包已编译。
- 从相邻 `himloco_custom_dog/assets/custom_dog_description` 准备模型 underlay。
  原始 URDF 字节保持一致，契约检查通过：13.84916 kg、12 关节、4 足端。
- 生成 `custom_dog_gazebo_point_foot.urdf`，仅移除 thigh/calf 碰撞体。
- 修正构建脚本的目录假设和加载 ROS 环境时的 `set -u` 问题。
- 为 Gazebo 控制插件添加显式 ROS 命名空间，并使用临时 ASCII 配置路径。
  修复前，SDF 把 `桌面` 解析为 `Lb`，控制器 YAML 无法打开并触发插件析构崩溃。
- 系统桌面已改名为 `Desktop`，控制工作区已在英文路径重新构建。
  `/home/hsc/桌面` 暂保留为兼容链接，供已有 IDE 会话和依赖构建中的绝对路径使用。

## 实测结果

测试使用 `ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1`，平地、默认仿真真值反馈。
运动测试与指令矩阵分别使用全新 Gazebo 世界；指令矩阵测试期间同时运行 GUI。

| 检查 | 结果 |
| --- | --- |
| 控制包测试 | 20 tests，0 errors，0 failures，0 skipped |
| 完整运动链 | PASSIVE → 起身 → MPC_STANCE → MPC_TROT → MPC_STANCE，通过 |
| 12 秒前进，指令 0.05 m/s | 前进 0.571 m，侧偏 -0.006 m |
| 上述运动最大 roll / pitch | 0.005 / 0.009 rad |
| 停车末速度 | x/y 均约 0 m/s |
| 矩阵前进，指令 0.05 m/s、8 秒 | dx +0.372 m，dy -0.002 m |
| 矩阵侧移，指令 0.04 m/s、8 秒 | dy +0.314 m，dx -0.012 m |
| 矩阵转向，指令 0.18 rad/s、8 秒 | yaw +1.444 rad |
| 三段矩阵的无效 WBC 样本 | 均为 0；每段停车均返回 MPC_STANCE |
| GUI | 模型及网格正常显示，站立截图已保存 |

测试结束后的诊断快照：机身高度 0.286864 m，控制周期 P99 1 ms，
NMPC 求解 P99 7.72051 ms，本次 WBC 求解 0.281804 ms。
这些是本机当前运行窗口的统计，不是硬实时保证。

本次未重新测试高速六方向包线、地形场景、关闭真值后的估计器闭环或实机。

## 复现

首次构建或更新后，使用 README 的一键构建入口。已构建时：

```bash
cd /home/hsc/Desktop/Dog/Dog-control
source src/custom_dog_control/scripts/env.sh
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1
ros2 launch custom_dog_control gazebo.launch.py use_rviz:=false
```

键盘输入发送到启动终端：`2` 起身，`W/S` 前后，`A/D` 侧移，
`J/L` 转向，空格停车，`Esc` 软件急停。关闭整个仿真使用启动终端的 `Ctrl+C`。

自动测试时用 `gui:=false use_rviz:=false start_keyboard:=false` 启动，
另一个终端 source 相同环境并设置相同 ROS_DOMAIN_ID，再分别冷启动运行：

```bash
ros2 run custom_dog_control simulation_motion_test.py
# 关闭并重新启动 Gazebo 后运行下一项
ros2 run custom_dog_control simulation_command_matrix_test.py
```

本次完整日志和截图在 `log/validation-20260923/`，该目录按仓库规则不纳入 Git。

## 工程化重构后的回归

同日完成控制器职责拆分、关节命令纯计算提取、共享环境入口与自动冷启动验收脚本。
参数与状态切换顺序保持原状；21 个原控制器方法中，19 个方法体保持一致，
另外 2 个方法仅把等效力矩和站立/WBC 交接计算提取为公共函数。

验证使用独立 `ROS_DOMAIN_ID=97`，自动脚本为两组场景分别创建 headless Gazebo，
使用默认真值反馈，退出后未残留 Gazebo 进程：

| 检查 | 结果 |
| --- | --- |
| 编译 | 3 个控制工作区包通过；模型 underlay 通过 |
| 单元及契约测试 | 6 个 CTest 入口，colcon 汇总 23 tests，0 errors / failures / skipped |
| 12 秒前进，0.05 m/s | dx +0.571 m，dy -0.001 m；最大 roll/pitch 0.004/0.009 rad；停车通过 |
| 矩阵前进 | dx +0.378 m，dy -0.003 m |
| 矩阵侧移 | dx -0.009 m，dy +0.315 m |
| 矩阵转向 | yaw +1.435 rad |
| 矩阵 WBC / 停车 | 每段 invalid_wbc=0，均恢复站立 |
| 模型测试路径发现 | 不传 URDF 路径时，从已安装 custom_dog_description 发现模型，配置及构建成功 |
| 环境脚本 | 普通及 nounset shell 均保留原选项；缺失模型环境返回 2 |
| 回归失败路径 | 人为设置启动超时 0.001 s，正确返回 1，写入失败 JSON 并清理进程 |
| 文档 / 脚本 | 本地文档链接、Bash 语法、Python 编译和 git diff 空白检查通过 |

复现：

```bash
CUSTOM_DOG_BUILD_ONLY=1 src/custom_dog_control/scripts/build_simulation.sh
source src/custom_dog_control/scripts/env.sh
colcon test --packages-select custom_dog_control --event-handlers console_direct+
colcon test-result --verbose
src/custom_dog_control/scripts/test_simulation.sh
```

本次运动日志与 JSON 汇总在 `log/refactor-validation-20260923/`；
有意触发超时的日志在 `log/refactor-timeout-validation-20260923/`。
构建仍有上游依赖和 Pinocchio 旧 API 的警告，未影响本次构建或仿真。
此次回归未重新验证高速包线、地形、非真值估计器闭环或实机。

## external 目录迁移

模型与 OCS2 已分别收拢到 `external/model_ws`、`external/ocs2_ws`。
旧工作区的源码、依赖补丁和历史日志随目录迁移；CMake/colcon 的构建和安装产物从新路径重建。
`external/COLCON_IGNORE` 隔离主工作区包扫描，Git 仅跟踪目录说明和此标记，不提交依赖副本。
本文前面的记录保留各阶段的实际环境；日常启动使用统一 `env.sh` 或 `run_simulation.sh`。

迁移后再次验证：

- 新路径下 OCS2 16 个包、模型包及主工作区 3 个包全部构建成功。
- 单元/契约测试仍为 23 tests，0 errors / failures / skipped。
- motion 冷启动通过：12 秒前进 dx +0.566 m，dy -0.001 m，最大 roll/pitch 0.003/0.010 rad，停车通过。
- matrix 冷启动通过：前进 dx +0.377 m、侧移 dy +0.318 m、转向 yaw +1.436 rad；每段 invalid_wbc=0，停车通过。
- 主工作区默认 colcon 扫描仅发现 custom_dog_control、fdilink_ahrs、serial。
- 新安装环境文件未发现旧中文路径或旧并列依赖工作区路径，ROS 包解析均指向新位置。
- 临时构建备份在验证通过后清理；仿真进程已正常退出。

日志位于 `log/external-migration-20260923/`，包括依赖/主工作区构建、单元测试、
两组冷启动仿真及 JSON 汇总；此目录按规则不纳入 Git。
