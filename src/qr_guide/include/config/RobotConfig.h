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

// 站立态力控试验参数。
// FixedStand 用 VMC 足端虚拟弹簧/阻尼计算支撑力，再经 Jacobian 转关节力矩。
struct HybridStandParameters {
    bool enabled = false;
    std::string force_mode = "vmc";
    double kp_z = 1200.0;
    double kd_z = 120.0;
    double kp_roll = 18.0;
    double kd_roll = 1.2;
    double kp_pitch = 18.0;
    double kd_pitch = 1.2;
    Vec3 vmc_kp_foot = Vec3(300.0, 300.0, 850.0);
    Vec3 vmc_kd_foot = Vec3(6.0, 6.0, 60.0);
    Vec3 vmc_error_limit_m = Vec3(0.03, 0.03, 0.04);
    double vmc_gravity_scale = 1.0;
    double vmc_pitch_load_shift_kp = 120.0;
    double vmc_pitch_load_shift_kd = 8.0;
    double vmc_pitch_load_shift_limit_n = 18.0;
    double vmc_lift_sync_load_kp = 220.0;
    double vmc_lift_sync_load_limit_n = 10.0;
    double vmc_lift_sync_z_gain = 0.30;
    double vmc_lift_sync_z_limit_m = 0.012;
    double vmc_min_cmd_kp = 0.0;
    double vmc_min_cmd_kd = 0.0;
    double vmc_q0_tau_limit_nm = 3.0;
    Vec3 vmc_joint_hold_kp = Vec3(7.3, 6.0, 6.0);
    Vec3 vmc_joint_hold_kd = Vec3(0.20, 0.25, 0.25);
    double vmc_handoff_min_height_m = 0.240;
    double vmc_handoff_max_rp_rad = 0.10;
    Vec3 vmc_handoff_max_foot_error_m = Vec3(0.025, 0.025, 0.035);
    Vec3 vmc_prehandoff_joint_kp = Vec3(7.3, 7.3, 7.3);
    Vec3 vmc_prehandoff_joint_kd = Vec3(0.20, 0.20, 0.20);
    double vmc_prehandoff_lift_scale = 1.25;
    double vmc_prehandoff_tau_limit_nm = 18.0;
    double fz_min_per_leg_n = 10.0;
    double fz_max_per_leg_n = 60.0;
    double tau_limit_nm = 6.0;
    double tau_rate_limit_nm_per_s = 120.0;
    double force_ramp_duration_s = 1.5;
    double max_force_scale = 1.0;
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

// 力控制参数（移植自 unitree_guide 的 BalanceCtrl + State_Trotting）。
// 所有 12 维向量（4 条腿 x 3 维力）用 3 维标量重复表示，简化 YAML 配置。
struct ForceControlParameters {
    // 机身 PD 增益
    Vec3 kp_body_xyz = Vec3(50, 50, 70);
    Vec3 kd_body_xyz = Vec3(8, 8, 10);
    Vec3 kp_body_rpy = Vec3(80, 80, 600);  // roll, pitch, yaw
    Vec3 kd_body_rpy = Vec3(50, 50, 50);

    // 摆动腿 PD 增益
    Vec3 kp_swing = Vec3(300, 300, 300);
    Vec3 kd_swing = Vec3(8, 8, 8);

    // QP 代价权重（简化为每组一个 3 维标量，内部自动复制到 4 条腿）
    Vec3 s_xyz = Vec3(20, 20, 50);      // 线加速度跟踪权重 (x, y, z)
    Vec3 s_rpy = Vec3(450, 450, 450);   // 角加速度跟踪权重 (roll, pitch, yaw)
    Vec3 w_per_foot = Vec3(10, 10, 4);  // 每条腿力正则化 (fx, fy, fz)
    Vec3 u_per_foot = Vec3(3, 3, 3);    // 每条腿力平滑 (fx, fy, fz)
    double alpha = 0.001;               // 力正则化系数
    double beta = 0.1;                  // 时间平滑系数
    double friction_ratio = 0.4;        // 摩擦系数

    // 安全限幅
    double tau_limit = 8.0;             // N·m
    double tau_rate_limit = 200.0;      // N·m/s
    double transition_steps = 100.0;    // 启动平滑步数

    // 期望加速度饱和
    Vec2 acc_xy_sat = Vec2(-3, 3);
    Vec2 acc_z_sat = Vec2(-5, 5);
    Vec2 w_roll_pitch_sat = Vec2(-40, 40);
    Vec2 w_yaw_sat = Vec2(-10, 10);
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
    ForceControlParameters force;
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
