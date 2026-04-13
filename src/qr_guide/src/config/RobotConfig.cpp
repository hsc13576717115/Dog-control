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
    }

    if (config["robot"]["velocity_limits"]) {
        params.velocity_limit_x = ReadVec2(config["robot"]["velocity_limits"]["x"]);
        params.velocity_limit_y = ReadVec2(config["robot"]["velocity_limits"]["y"]);
        params.velocity_limit_yaw = ReadVec2(config["robot"]["velocity_limits"]["yaw"]);
    }

    ValidateRightPositiveYConvention(params);
    return params;
}

}  // namespace qr_guide
