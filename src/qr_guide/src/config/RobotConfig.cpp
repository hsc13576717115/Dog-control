#include "config/RobotConfig.h"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace qr_guide {

namespace {

// 读取三维向量参数，失败时直接抛异常，避免静默使用错误配置。
Vec3 ReadVec3(const YAML::Node& node) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        throw std::runtime_error("Expected a 3-element sequence in robot config.");
    }
    Vec3 value;
    value << node[0].as<double>(), node[1].as<double>(), node[2].as<double>();
    return value;
}

// 读取二维上下限参数。
Vec2 ReadVec2(const YAML::Node& node) {
    if (!node || !node.IsSequence() || node.size() != 2) {
        throw std::runtime_error("Expected a 2-element sequence in robot config.");
    }
    Vec2 value;
    value << node[0].as<double>(), node[1].as<double>();
    return value;
}

// 按固定腿序读取 FR / FL / RR / RL 四条腿对应的三维向量。
std::array<Vec3, NumLeg> ReadLegVec3Map(const YAML::Node& node) {
    std::array<Vec3, NumLeg> values;
    values[FR] = ReadVec3(node["FR"]);
    values[FL] = ReadVec3(node["FL"]);
    values[RR] = ReadVec3(node["RR"]);
    values[RL] = ReadVec3(node["RL"]);
    return values;
}

// 按固定腿序读取 FR / FL / RR / RL 四条腿对应的字符串。
std::array<std::string, NumLeg> ReadLegStringMap(const YAML::Node& node) {
    std::array<std::string, NumLeg> values;
    values[FR] = node["FR"].as<std::string>();
    values[FL] = node["FL"].as<std::string>();
    values[RR] = node["RR"].as<std::string>();
    values[RL] = node["RL"].as<std::string>();
    return values;
}

void ValidateRightPositiveYConvention(const RobotParameters& params) {
    const auto require_leg_side_sign = [&](const std::array<Vec3, NumLeg>& values,
                                           const char* field_name) {
        if (values[FR].y() <= 0.0 || values[RR].y() <= 0.0 ||
            values[FL].y() >= 0.0 || values[RL].y() >= 0.0) {
            throw std::runtime_error(
                std::string("Invalid coordinate convention in ") + field_name +
                ": expected FR/RR y > 0 and FL/RL y < 0 under the unified body/hip frame.");
        }
    };

    require_leg_side_sign(params.hip_mounts_in_body, "hip_mounts_in_body");
    require_leg_side_sign(params.stand_targets.normal_feet_in_hip, "stand_targets.normal_feet_in_hip");
    require_leg_side_sign(params.stand_targets.crouch_feet_in_hip, "stand_targets.crouch_feet_in_hip");
}

}  // namespace

Vec3 JointLimitParameters::lower() const {
    return Vec3(q0(0), q1(0), q2(0));
}

Vec3 JointLimitParameters::upper() const {
    return Vec3(q0(1), q1(1), q2(1));
}

Vec34 RobotParameters::hipMountMatrix() const {
    Vec34 mounts;
    for (int leg = 0; leg < NumLeg; ++leg) {
        mounts.col(leg) = hip_mounts_in_body[leg];
    }
    return mounts;
}

Vec34 RobotParameters::normalStandFeetInBody() const {
    Vec34 feet;
    for (int leg = 0; leg < NumLeg; ++leg) {
        feet.col(leg) = hip_mounts_in_body[leg] + stand_targets.normal_feet_in_hip[leg];
    }
    return feet;
}

RobotParameters LoadRobotParameters(const std::string& path) {
    const YAML::Node config = YAML::LoadFile(path);
    RobotParameters params;

    // 所有主控制参数都从这一个文件里统一读取，避免多处硬编码。
    params.name = config["robot"]["name"].as<std::string>();
    params.body_size_m = ReadVec3(config["robot"]["body_size_m"]);
    params.mass.total_mass_kg = config["robot"]["mass"]["total_mass_kg"].as<double>();
    params.mass.body_mass_kg = config["robot"]["mass"]["body_mass_kg"].as<double>();
    params.mass.leg_mass_each_kg = config["robot"]["mass"]["leg_mass_each_kg"].as<double>();
    params.mass.thigh_mass_each_kg = config["robot"]["mass"]["thigh_mass_each_kg"].as<double>();
    params.mass.calf_mass_each_kg = config["robot"]["mass"]["calf_mass_each_kg"].as<double>();
    params.mass.com_offset_m = ReadVec3(config["robot"]["mass"]["com_offset_m"]);
    params.mass.body_inertia_kg_m2_diag =
        ReadVec3(config["robot"]["mass"]["body_inertia_kg_m2_diag"]);
    params.mass.whole_robot_inertia_kg_m2_diag =
        ReadVec3(config["robot"]["mass"]["whole_robot_inertia_kg_m2_diag"]);

    params.hip_mounts_in_body = ReadLegVec3Map(config["robot"]["hip_mounts_in_body"]);
    params.l0 = config["robot"]["leg_geometry"]["l0"].as<double>();
    params.l1 = config["robot"]["leg_geometry"]["l1"].as<double>();
    params.l2 = config["robot"]["leg_geometry"]["l2"].as<double>();
    if (config["robot"]["leg_geometry"]["foot_radius_m"]) {
        params.foot_radius_m = config["robot"]["leg_geometry"]["foot_radius_m"].as<double>();
    }

    params.drive.serial_ports = ReadLegStringMap(config["robot"]["drive"]["serial_ports"]);
    params.drive.base_gear_ratio = config["robot"]["drive"]["base_gear_ratio"].as<double>();
    params.drive.calf_total_gear_ratio =
        config["robot"]["drive"]["calf_total_gear_ratio"].as<double>();
    if (config["robot"]["drive"]["use_parallel_leg_io"]) {
        params.drive.use_parallel_leg_io =
            config["robot"]["drive"]["use_parallel_leg_io"].as<bool>();
    }

    params.joint_limits.q0 = ReadVec2(config["robot"]["joint_limits"]["q0"]);
    params.joint_limits.q1 = ReadVec2(config["robot"]["joint_limits"]["q1"]);
    params.joint_limits.q2 = ReadVec2(config["robot"]["joint_limits"]["q2"]);

    params.stand_targets.normal_feet_in_hip =
        ReadLegVec3Map(config["robot"]["stand_targets"]["normal_feet_in_hip"]);
    params.stand_targets.crouch_feet_in_hip =
        ReadLegVec3Map(config["robot"]["stand_targets"]["crouch_feet_in_hip"]);

    if (const YAML::Node hybrid = config["robot"]["hybrid_stand"]) {
        if (hybrid["enabled"]) {
            params.hybrid_stand.enabled = hybrid["enabled"].as<bool>();
        }
        if (hybrid["force_mode"]) {
            params.hybrid_stand.force_mode = hybrid["force_mode"].as<std::string>();
        }
        if (hybrid["kp_z"]) {
            params.hybrid_stand.kp_z = hybrid["kp_z"].as<double>();
        }
        if (hybrid["kd_z"]) {
            params.hybrid_stand.kd_z = hybrid["kd_z"].as<double>();
        }
        if (hybrid["kp_roll"]) {
            params.hybrid_stand.kp_roll = hybrid["kp_roll"].as<double>();
        }
        if (hybrid["kd_roll"]) {
            params.hybrid_stand.kd_roll = hybrid["kd_roll"].as<double>();
        }
        if (hybrid["kp_pitch"]) {
            params.hybrid_stand.kp_pitch = hybrid["kp_pitch"].as<double>();
        }
        if (hybrid["kd_pitch"]) {
            params.hybrid_stand.kd_pitch = hybrid["kd_pitch"].as<double>();
        }
        if (hybrid["vmc_kp_foot"]) {
            params.hybrid_stand.vmc_kp_foot = ReadVec3(hybrid["vmc_kp_foot"]);
        }
        if (hybrid["vmc_kd_foot"]) {
            params.hybrid_stand.vmc_kd_foot = ReadVec3(hybrid["vmc_kd_foot"]);
        }
        if (hybrid["vmc_error_limit_m"]) {
            params.hybrid_stand.vmc_error_limit_m = ReadVec3(hybrid["vmc_error_limit_m"]);
        }
        if (hybrid["vmc_gravity_scale"]) {
            params.hybrid_stand.vmc_gravity_scale = hybrid["vmc_gravity_scale"].as<double>();
        }
        if (hybrid["vmc_pitch_load_shift_kp"]) {
            params.hybrid_stand.vmc_pitch_load_shift_kp =
                hybrid["vmc_pitch_load_shift_kp"].as<double>();
        }
        if (hybrid["vmc_pitch_load_shift_kd"]) {
            params.hybrid_stand.vmc_pitch_load_shift_kd =
                hybrid["vmc_pitch_load_shift_kd"].as<double>();
        }
        if (hybrid["vmc_pitch_load_shift_limit_n"]) {
            params.hybrid_stand.vmc_pitch_load_shift_limit_n =
                hybrid["vmc_pitch_load_shift_limit_n"].as<double>();
        }
        if (hybrid["vmc_lift_sync_load_kp"]) {
            params.hybrid_stand.vmc_lift_sync_load_kp =
                hybrid["vmc_lift_sync_load_kp"].as<double>();
        }
        if (hybrid["vmc_lift_sync_load_limit_n"]) {
            params.hybrid_stand.vmc_lift_sync_load_limit_n =
                hybrid["vmc_lift_sync_load_limit_n"].as<double>();
        }
        if (hybrid["vmc_lift_sync_z_gain"]) {
            params.hybrid_stand.vmc_lift_sync_z_gain =
                hybrid["vmc_lift_sync_z_gain"].as<double>();
        }
        if (hybrid["vmc_lift_sync_z_limit_m"]) {
            params.hybrid_stand.vmc_lift_sync_z_limit_m =
                hybrid["vmc_lift_sync_z_limit_m"].as<double>();
        }
        if (hybrid["vmc_min_cmd_kp"]) {
            params.hybrid_stand.vmc_min_cmd_kp = hybrid["vmc_min_cmd_kp"].as<double>();
        }
        if (hybrid["vmc_min_cmd_kd"]) {
            params.hybrid_stand.vmc_min_cmd_kd = hybrid["vmc_min_cmd_kd"].as<double>();
        }
        if (hybrid["vmc_q0_tau_limit_nm"]) {
            params.hybrid_stand.vmc_q0_tau_limit_nm =
                hybrid["vmc_q0_tau_limit_nm"].as<double>();
        }
        if (hybrid["vmc_joint_hold_kp"]) {
            params.hybrid_stand.vmc_joint_hold_kp = ReadVec3(hybrid["vmc_joint_hold_kp"]);
        }
        if (hybrid["vmc_joint_hold_kd"]) {
            params.hybrid_stand.vmc_joint_hold_kd = ReadVec3(hybrid["vmc_joint_hold_kd"]);
        }
        if (hybrid["vmc_handoff_min_height_m"]) {
            params.hybrid_stand.vmc_handoff_min_height_m =
                hybrid["vmc_handoff_min_height_m"].as<double>();
        }
        if (hybrid["vmc_handoff_max_rp_rad"]) {
            params.hybrid_stand.vmc_handoff_max_rp_rad =
                hybrid["vmc_handoff_max_rp_rad"].as<double>();
        }
        if (hybrid["vmc_handoff_max_foot_error_m"]) {
            params.hybrid_stand.vmc_handoff_max_foot_error_m =
                ReadVec3(hybrid["vmc_handoff_max_foot_error_m"]);
        }
        if (hybrid["vmc_prehandoff_joint_kp"]) {
            params.hybrid_stand.vmc_prehandoff_joint_kp =
                ReadVec3(hybrid["vmc_prehandoff_joint_kp"]);
        }
        if (hybrid["vmc_prehandoff_joint_kd"]) {
            params.hybrid_stand.vmc_prehandoff_joint_kd =
                ReadVec3(hybrid["vmc_prehandoff_joint_kd"]);
        }
        if (hybrid["vmc_prehandoff_lift_scale"]) {
            params.hybrid_stand.vmc_prehandoff_lift_scale =
                hybrid["vmc_prehandoff_lift_scale"].as<double>();
        }
        if (hybrid["vmc_prehandoff_tau_limit_nm"]) {
            params.hybrid_stand.vmc_prehandoff_tau_limit_nm =
                hybrid["vmc_prehandoff_tau_limit_nm"].as<double>();
        }
        if (hybrid["fz_min_per_leg_n"]) {
            params.hybrid_stand.fz_min_per_leg_n = hybrid["fz_min_per_leg_n"].as<double>();
        }
        if (hybrid["fz_max_per_leg_n"]) {
            params.hybrid_stand.fz_max_per_leg_n = hybrid["fz_max_per_leg_n"].as<double>();
        }
        if (hybrid["tau_limit_nm"]) {
            params.hybrid_stand.tau_limit_nm = hybrid["tau_limit_nm"].as<double>();
        }
        if (hybrid["tau_rate_limit_nm_per_s"]) {
            params.hybrid_stand.tau_rate_limit_nm_per_s =
                hybrid["tau_rate_limit_nm_per_s"].as<double>();
        }
        if (hybrid["force_ramp_duration_s"]) {
            params.hybrid_stand.force_ramp_duration_s =
                hybrid["force_ramp_duration_s"].as<double>();
        }
        if (hybrid["max_force_scale"]) {
            params.hybrid_stand.max_force_scale = hybrid["max_force_scale"].as<double>();
        }
    }

    if (const YAML::Node force = config["robot"]["force_control"]) {
        if (force["kp_body_xyz"]) {
            params.force.kp_body_xyz = ReadVec3(force["kp_body_xyz"]);
        }
        if (force["kd_body_xyz"]) {
            params.force.kd_body_xyz = ReadVec3(force["kd_body_xyz"]);
        }
        if (force["kp_body_rpy"]) {
            params.force.kp_body_rpy = ReadVec3(force["kp_body_rpy"]);
        }
        if (force["kd_body_rpy"]) {
            params.force.kd_body_rpy = ReadVec3(force["kd_body_rpy"]);
        }
        if (force["kp_swing"]) {
            params.force.kp_swing = ReadVec3(force["kp_swing"]);
        }
        if (force["kd_swing"]) {
            params.force.kd_swing = ReadVec3(force["kd_swing"]);
        }
        if (force["s_xyz"]) {
            params.force.s_xyz = ReadVec3(force["s_xyz"]);
        }
        if (force["s_rpy"]) {
            params.force.s_rpy = ReadVec3(force["s_rpy"]);
        }
        if (force["w_per_foot"]) {
            params.force.w_per_foot = ReadVec3(force["w_per_foot"]);
        }
        if (force["u_per_foot"]) {
            params.force.u_per_foot = ReadVec3(force["u_per_foot"]);
        }
        if (force["alpha"]) {
            params.force.alpha = force["alpha"].as<double>();
        }
        if (force["beta"]) {
            params.force.beta = force["beta"].as<double>();
        }
        if (force["friction_ratio"]) {
            params.force.friction_ratio = force["friction_ratio"].as<double>();
        }
        if (force["tau_limit"]) {
            params.force.tau_limit = force["tau_limit"].as<double>();
        }
        if (force["tau_rate_limit"]) {
            params.force.tau_rate_limit = force["tau_rate_limit"].as<double>();
        }
        if (force["transition_steps"]) {
            params.force.transition_steps = force["transition_steps"].as<double>();
        }
        if (force["acc_xy_sat"]) {
            params.force.acc_xy_sat = ReadVec2(force["acc_xy_sat"]);
        }
        if (force["acc_z_sat"]) {
            params.force.acc_z_sat = ReadVec2(force["acc_z_sat"]);
        }
        if (force["w_roll_pitch_sat"]) {
            params.force.w_roll_pitch_sat = ReadVec2(force["w_roll_pitch_sat"]);
        }
        if (force["w_yaw_sat"]) {
            params.force.w_yaw_sat = ReadVec2(force["w_yaw_sat"]);
        }
    }

    if (config["robot"]["velocity_limits"]) {
        params.velocity_limit_x = ReadVec2(config["robot"]["velocity_limits"]["x"]);
        params.velocity_limit_y = ReadVec2(config["robot"]["velocity_limits"]["y"]);
        params.velocity_limit_yaw = ReadVec2(config["robot"]["velocity_limits"]["yaw"]);
    }

    if (const YAML::Node joy_mapping = config["robot"]["joy_mapping"]) {
        if (joy_mapping["deadband"]) {
            params.joy_mapping.deadband = joy_mapping["deadband"].as<double>();
        }
        if (joy_mapping["expo"]) {
            params.joy_mapping.expo = joy_mapping["expo"].as<double>();
        }
        if (joy_mapping["max_vx"]) {
            params.joy_mapping.max_vx = joy_mapping["max_vx"].as<double>();
        }
        if (joy_mapping["max_vy"]) {
            params.joy_mapping.max_vy = joy_mapping["max_vy"].as<double>();
        }
        if (joy_mapping["max_yaw"]) {
            params.joy_mapping.max_yaw = joy_mapping["max_yaw"].as<double>();
        }
        if (joy_mapping["max_accel_x"]) {
            params.joy_mapping.max_accel_x = joy_mapping["max_accel_x"].as<double>();
        }
        if (joy_mapping["max_accel_y"]) {
            params.joy_mapping.max_accel_y = joy_mapping["max_accel_y"].as<double>();
        }
        if (joy_mapping["max_accel_yaw"]) {
            params.joy_mapping.max_accel_yaw = joy_mapping["max_accel_yaw"].as<double>();
        }
    } else {
        params.joy_mapping.max_vx = params.velocity_limit_x(1);
        params.joy_mapping.max_vy = params.velocity_limit_y(1);
        params.joy_mapping.max_yaw = params.velocity_limit_yaw(1);
    }

    if (const YAML::Node trot = config["robot"]["trot"]) {
        if (trot["force_mode"]) {
            params.trot.force_mode = trot["force_mode"].as<std::string>();
        }
        if (trot["gait_start_command_eps"]) {
            params.trot.gait_start_command_eps = trot["gait_start_command_eps"].as<double>();
        }
        if (trot["gait_stop_command_eps"]) {
            params.trot.gait_stop_command_eps = trot["gait_stop_command_eps"].as<double>();
        }
        if (trot["gait_start_debounce_s"]) {
            params.trot.gait_start_debounce_s = trot["gait_start_debounce_s"].as<double>();
        }
        if (trot["gait_ramp_cycles"]) {
            params.trot.gait_ramp_cycles = trot["gait_ramp_cycles"].as<double>();
        }
        if (trot["cycle_time"]) {
            params.trot.cycle_time = trot["cycle_time"].as<double>();
        }
        if (trot["stance_ratio"]) {
            params.trot.stance_ratio = trot["stance_ratio"].as<double>();
        }
        if (trot["lift_height"]) {
            params.trot.lift_height = trot["lift_height"].as<double>();
        }
        if (trot["swing_lift_peak_phase"]) {
            params.trot.swing_lift_peak_phase = trot["swing_lift_peak_phase"].as<double>();
        }
        if (trot["landing_preview_gain"]) {
            params.trot.landing_preview_gain = trot["landing_preview_gain"].as<double>();
        }
        if (trot["vel_error_gain"]) {
            params.trot.vel_error_gain = trot["vel_error_gain"].as<double>();
        }
        if (trot["vmc_kp_foot"]) {
            params.trot.vmc_kp_foot = ReadVec3(trot["vmc_kp_foot"]);
        }
        if (trot["vmc_kd_foot"]) {
            params.trot.vmc_kd_foot = ReadVec3(trot["vmc_kd_foot"]);
        }
        if (trot["vmc_error_limit_m"]) {
            params.trot.vmc_error_limit_m = ReadVec3(trot["vmc_error_limit_m"]);
        }
        if (trot["vmc_q0_tau_limit_nm"]) {
            params.trot.vmc_q0_tau_limit_nm = trot["vmc_q0_tau_limit_nm"].as<double>();
        }
        if (trot["active_vmc_kp_foot"]) {
            params.trot.active_vmc_kp_foot = ReadVec3(trot["active_vmc_kp_foot"]);
        }
        if (trot["active_vmc_kd_foot"]) {
            params.trot.active_vmc_kd_foot = ReadVec3(trot["active_vmc_kd_foot"]);
        }
        if (trot["active_vmc_error_limit_m"]) {
            params.trot.active_vmc_error_limit_m = ReadVec3(trot["active_vmc_error_limit_m"]);
        }
        if (trot["active_vmc_q0_tau_limit_nm"]) {
            params.trot.active_vmc_q0_tau_limit_nm =
                trot["active_vmc_q0_tau_limit_nm"].as<double>();
        }
        if (trot["stance_trajectory_gain"]) {
            params.trot.stance_trajectory_gain = trot["stance_trajectory_gain"].as<double>();
        }
        if (trot["vmc_attitude_fz_gain"]) {
            params.trot.vmc_attitude_fz_gain = trot["vmc_attitude_fz_gain"].as<double>();
        }
        if (trot["vmc_attitude_fz_limit_n"]) {
            params.trot.vmc_attitude_fz_limit_n = trot["vmc_attitude_fz_limit_n"].as<double>();
        }
        if (trot["vmc_force_rate_limit_n_per_s"]) {
            params.trot.vmc_force_rate_limit_n_per_s =
                trot["vmc_force_rate_limit_n_per_s"].as<double>();
        }
        if (trot["active_tau_rate_limit_nm_per_s"]) {
            params.trot.active_tau_rate_limit_nm_per_s =
                trot["active_tau_rate_limit_nm_per_s"].as<double>();
        }
        if (trot["active_rear_fz_boost_n"]) {
            params.trot.active_rear_fz_boost_n =
                trot["active_rear_fz_boost_n"].as<double>();
        }
        if (trot["active_diagonal_fz_balance_gain"]) {
            params.trot.active_diagonal_fz_balance_gain =
                trot["active_diagonal_fz_balance_gain"].as<double>();
        }
        if (trot["idle_fz_min_per_leg_n"]) {
            params.trot.idle_fz_min_per_leg_n =
                trot["idle_fz_min_per_leg_n"].as<double>();
        }
        if (trot["active_fz_min_per_leg_n"]) {
            params.trot.active_fz_min_per_leg_n =
                trot["active_fz_min_per_leg_n"].as<double>();
        }
        if (trot["idle_fz_max_per_leg_n"]) {
            params.trot.idle_fz_max_per_leg_n =
                trot["idle_fz_max_per_leg_n"].as<double>();
        }
        if (trot["active_fz_max_per_leg_n"]) {
            params.trot.active_fz_max_per_leg_n =
                trot["active_fz_max_per_leg_n"].as<double>();
        }
        if (trot["idle_settle_duration_s"]) {
            params.trot.idle_settle_duration_s =
                trot["idle_settle_duration_s"].as<double>();
        }
        if (trot["planar_vmc_kp_x"]) {
            params.trot.planar_vmc_kp_x = trot["planar_vmc_kp_x"].as<double>();
        }
        if (trot["planar_vmc_kd_x"]) {
            params.trot.planar_vmc_kd_x = trot["planar_vmc_kd_x"].as<double>();
        }
        if (trot["planar_vmc_kp_z"]) {
            params.trot.planar_vmc_kp_z = trot["planar_vmc_kp_z"].as<double>();
        }
        if (trot["planar_vmc_kd_z"]) {
            params.trot.planar_vmc_kd_z = trot["planar_vmc_kd_z"].as<double>();
        }
        if (trot["planar_vmc_error_limit_x_m"]) {
            params.trot.planar_vmc_error_limit_x_m =
                trot["planar_vmc_error_limit_x_m"].as<double>();
        }
        if (trot["planar_vmc_error_limit_z_m"]) {
            params.trot.planar_vmc_error_limit_z_m =
                trot["planar_vmc_error_limit_z_m"].as<double>();
        }
        if (trot["planar_vmc_tau_limit_nm"]) {
            params.trot.planar_vmc_tau_limit_nm =
                trot["planar_vmc_tau_limit_nm"].as<double>();
        }
        if (trot["planar_q0_kp"]) {
            params.trot.planar_q0_kp = trot["planar_q0_kp"].as<double>();
        }
        if (trot["planar_q0_kd"]) {
            params.trot.planar_q0_kd = trot["planar_q0_kd"].as<double>();
        }
        if (trot["stance_joint_kp"]) {
            params.trot.stance_joint_kp = ReadVec3(trot["stance_joint_kp"]);
        }
        if (trot["stance_joint_kd"]) {
            params.trot.stance_joint_kd = ReadVec3(trot["stance_joint_kd"]);
        }
        if (trot["active_stance_joint_kp"]) {
            params.trot.active_stance_joint_kp = ReadVec3(trot["active_stance_joint_kp"]);
        }
        if (trot["active_stance_joint_kd"]) {
            params.trot.active_stance_joint_kd = ReadVec3(trot["active_stance_joint_kd"]);
        }
        if (trot["swing_joint_kp"]) {
            params.trot.swing_joint_kp = ReadVec3(trot["swing_joint_kp"]);
        }
        if (trot["swing_joint_kd"]) {
            params.trot.swing_joint_kd = ReadVec3(trot["swing_joint_kd"]);
        }
        if (trot["landing_vel_error_gain"]) {
            params.trot.landing_vel_error_gain = trot["landing_vel_error_gain"].as<double>();
        }
        if (trot["landing_yaw_error_gain"]) {
            params.trot.landing_yaw_error_gain = trot["landing_yaw_error_gain"].as<double>();
        }
        if (trot["pure_yaw_threshold"]) {
            params.trot.pure_yaw_threshold = trot["pure_yaw_threshold"].as<double>();
        }
        if (trot["pure_rotation_translation_eps"]) {
            params.trot.pure_rotation_translation_eps =
                trot["pure_rotation_translation_eps"].as<double>();
        }
        if (trot["max_foothold_shift_x"]) {
            params.trot.max_foothold_shift_x = trot["max_foothold_shift_x"].as<double>();
        }
        if (trot["max_foothold_shift_y"]) {
            params.trot.max_foothold_shift_y = trot["max_foothold_shift_y"].as<double>();
        }
        if (trot["lateral_min_foot_y_m"]) {
            params.trot.lateral_min_foot_y_m = trot["lateral_min_foot_y_m"].as<double>();
        }
        if (trot["lateral_roll_foothold_gain"]) {
            params.trot.lateral_roll_foothold_gain =
                trot["lateral_roll_foothold_gain"].as<double>();
        }
        if (trot["lateral_gyro_foothold_gain"]) {
            params.trot.lateral_gyro_foothold_gain =
                trot["lateral_gyro_foothold_gain"].as<double>();
        }
        if (trot["lateral_vy_foothold_gain"]) {
            params.trot.lateral_vy_foothold_gain =
                trot["lateral_vy_foothold_gain"].as<double>();
        }
        if (trot["lateral_foothold_extra_limit_m"]) {
            params.trot.lateral_foothold_extra_limit_m =
                trot["lateral_foothold_extra_limit_m"].as<double>();
        }
        if (trot["max_joint_delta"]) {
            params.trot.max_joint_delta = trot["max_joint_delta"].as<double>();
        }
        if (trot["debug_print_period_s"]) {
            params.trot.debug_print_period_s = trot["debug_print_period_s"].as<double>();
        }
    }

    if (const YAML::Node stand = config["robot"]["stand"]) {
        if (stand["entry_duration"]) {
            params.stand.entry_duration = stand["entry_duration"].as<int>();
        }
        if (stand["body_height"]) {
            params.stand.body_height = stand["body_height"].as<double>();
        }
        if (stand["roll_pitch_comp_gain"]) {
            params.stand.roll_pitch_comp_gain = stand["roll_pitch_comp_gain"].as<double>();
        }
        if (stand["height_comp_gain"]) {
            params.stand.height_comp_gain = stand["height_comp_gain"].as<double>();
        }
        if (stand["compensation_limit"]) {
            params.stand.compensation_limit = stand["compensation_limit"].as<double>();
        }
    }

    ValidateRightPositiveYConvention(params);
    return params;
}

}  // namespace qr_guide
