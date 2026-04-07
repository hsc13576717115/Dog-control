#ifndef QR_GUIDE_CONFIG_ROBOTCONFIG_H
#define QR_GUIDE_CONFIG_ROBOTCONFIG_H

#include <array>
#include <string>

#include "common/mathTypes.h"

namespace qr_guide {

// 腿序在整个重构后统一为 FR / FL / RR / RL。
// 坐标系约定也统一为：
// - body / hip frame: x 向前, y 向机器人右侧, z 向上
// - 因此 FR / RR 的 y 为正，FL / RL 的 y 为负
enum LegIndex {
    FR = 0,
    FL = 1,
    RR = 2,
    RL = 3,
    NumLeg = 4,
};

// 质量、COM 和惯量参数。
struct MassParameters {
    double total_mass_kg = 0.0;
    double body_mass_kg = 0.0;
    double leg_mass_each_kg = 0.0;
    double thigh_mass_each_kg = 0.0;
    double calf_mass_each_kg = 0.0;
    Vec3 com_offset_m = Vec3::Zero();
    Vec3 body_inertia_kg_m2_diag = Vec3::Zero();
    Vec3 whole_robot_inertia_kg_m2_diag = Vec3::Zero();
};

// 与真实电机链路相关的串口和减速比参数。
struct DriveParameters {
    std::array<std::string, NumLeg> serial_ports;
    double base_gear_ratio = 0.0;
    double calf_total_gear_ratio = 0.0;
};

// 三个关节的机械限位。
struct JointLimitParameters {
    Vec2 q0 = Vec2::Zero();
    Vec2 q1 = Vec2::Zero();
    Vec2 q2 = Vec2::Zero();

    Vec3 lower() const;
    Vec3 upper() const;
};

// 常用站姿对应的足端目标，统一定义在 hip frame 下。
// 同样采用 x 向前, y 向右, z 向上的约定。
struct StandTargetParameters {
    std::array<Vec3, NumLeg> normal_feet_in_hip;
    std::array<Vec3, NumLeg> crouch_feet_in_hip;
};

// 机器人主参数结构，作为整机模型和控制器的统一配置入口。
struct RobotParameters {
    std::string name;
    Vec3 body_size_m = Vec3::Zero();
    MassParameters mass;
    // 髋安装点定义在 body frame 下，约定 FR / RR 的 y 为正。
    std::array<Vec3, NumLeg> hip_mounts_in_body;
    double l0 = 0.0;
    double l1 = 0.0;
    double l2 = 0.0;
    DriveParameters drive;
    JointLimitParameters joint_limits;
    StandTargetParameters stand_targets;
    Vec2 velocity_limit_x = Vec2(-0.4, 0.4);
    Vec2 velocity_limit_y = Vec2(-0.3, 0.3);
    Vec2 velocity_limit_yaw = Vec2(-0.5, 0.5);

    Vec34 hipMountMatrix() const;
    Vec34 normalStandFeetInBody() const;
};

// 从 YAML 文件加载机器人参数。
RobotParameters LoadRobotParameters(const std::string& path);
// 保留 A1 / Go1 参数工厂，主要用于兼容旧接口。
RobotParameters MakeA1Parameters();
RobotParameters MakeGo1Parameters();

}  // namespace qr_guide

#endif  // QR_GUIDE_CONFIG_ROBOTCONFIG_H
