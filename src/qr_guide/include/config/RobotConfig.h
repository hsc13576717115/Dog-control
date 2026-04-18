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
    bool use_parallel_leg_io = false;
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

// 站立态力位混合控制参数。
// 第一版只用于 FixedStand 的 z / roll / pitch 三个任务。
struct HybridStandParameters {
    bool enabled = false;
    double kp_z = 1200.0;
    double kd_z = 120.0;
    double kp_roll = 18.0;
    double kd_roll = 1.2;
    double kp_pitch = 18.0;
    double kd_pitch = 1.2;
    double fz_min_per_leg_n = 10.0;
    double fz_max_per_leg_n = 60.0;
    double tau_limit_nm = 6.0;
    double tau_rate_limit_nm_per_s = 120.0;
};

// 手柄速度映射参数。
// 第一阶段统一约定为：死区 -> 非线性整形 -> 加速度限幅 -> 机体系限幅。
struct JoyMappingParameters {
    double deadband = 0.08;
    double expo = 3.0;
    double max_vx = 0.4;
    double max_vy = 0.3;
    double max_yaw = 0.5;
    double max_accel_x = 1.2;
    double max_accel_y = 0.9;
    double max_accel_yaw = 2.0;
};

// trot 轨迹参数。
// 这里仅覆盖第一阶段的几何轨迹与落脚点预瞄，不涉及力控。
struct TrotParameters {
    double cycle_time = 0.35;
    double stance_ratio = 0.5;
    double lift_height = 0.10;
    double landing_preview_gain = 0.5;
    double vel_error_gain = 0.05;
    double pure_yaw_threshold = 0.10;
    double pure_rotation_translation_eps = 0.05;
    double max_foothold_shift_x = 0.08;
    double max_foothold_shift_y = 0.05;
    double max_joint_delta = 0.03;
};

// 纯位置站立稳定化参数。
// 第一阶段只允许足端空间补偿，不引入力矩前馈。
struct StandControlParameters {
    int entry_duration = 500;
    double body_height = 0.24;
    double roll_pitch_comp_gain = 0.08;
    double height_comp_gain = 0.15;
    double compensation_limit = 0.02;
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
    double foot_radius_m = 0.0;
    DriveParameters drive;
    JointLimitParameters joint_limits;
    StandTargetParameters stand_targets;
    HybridStandParameters hybrid_stand;
    JoyMappingParameters joy_mapping;
    TrotParameters trot;
    StandControlParameters stand;
    Vec2 velocity_limit_x = Vec2(-0.4, 0.4);
    Vec2 velocity_limit_y = Vec2(-0.3, 0.3);
    Vec2 velocity_limit_yaw = Vec2(-0.5, 0.5);

    Vec34 hipMountMatrix() const;
    Vec34 normalStandFeetInBody() const;
};

// 从 YAML 文件加载机器人参数。
RobotParameters LoadRobotParameters(const std::string& path);

}  // namespace qr_guide

#endif  // QR_GUIDE_CONFIG_ROBOTCONFIG_H
