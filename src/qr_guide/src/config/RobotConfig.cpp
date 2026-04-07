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

    if (config["robot"]["velocity_limits"]) {
        params.velocity_limit_x = ReadVec2(config["robot"]["velocity_limits"]["x"]);
        params.velocity_limit_y = ReadVec2(config["robot"]["velocity_limits"]["y"]);
        params.velocity_limit_yaw = ReadVec2(config["robot"]["velocity_limits"]["yaw"]);
    }

    ValidateRightPositiveYConvention(params);
    return params;
}

RobotParameters MakeA1Parameters() {
    RobotParameters params;
    // 兼容旧接口用的默认参数，不参与当前 custom_quadruped 主线。
    params.name = "A1";
    params.body_size_m << 0.267, 0.194, 0.114;
    params.mass.total_mass_kg = 12.5;
    params.mass.body_mass_kg = 6.0;
    params.mass.com_offset_m << 0.01, 0.0, 0.0;
    params.mass.body_inertia_kg_m2_diag << 0.0158533, 0.0377999, 0.0456542;
    params.mass.whole_robot_inertia_kg_m2_diag << 0.132, 0.3475, 0.3775;
    params.hip_mounts_in_body[FR] << 0.1805, 0.0470, 0.0;
    params.hip_mounts_in_body[FL] << 0.1805, -0.0470, 0.0;
    params.hip_mounts_in_body[RR] << -0.1805, 0.0470, 0.0;
    params.hip_mounts_in_body[RL] << -0.1805, -0.0470, 0.0;
    params.l0 = 0.0838;
    params.l1 = 0.2000;
    params.l2 = 0.2000;
    params.joint_limits.q0 << -2.6, 2.6;
    params.joint_limits.q1 << -6.5, 6.5;
    params.joint_limits.q2 << -2.3, 2.3;
    params.stand_targets.normal_feet_in_hip[FR] << 0.0, 0.1308, -0.3180;
    params.stand_targets.normal_feet_in_hip[FL] << 0.0, -0.1308, -0.3180;
    params.stand_targets.normal_feet_in_hip[RR] << 0.0, 0.1308, -0.3180;
    params.stand_targets.normal_feet_in_hip[RL] << 0.0, -0.1308, -0.3180;
    params.stand_targets.crouch_feet_in_hip = params.stand_targets.normal_feet_in_hip;
    ValidateRightPositiveYConvention(params);
    return params;
}

RobotParameters MakeGo1Parameters() {
    RobotParameters params;
    // 兼容旧接口用的默认参数，不参与当前 custom_quadruped 主线。
    params.name = "Go1";
    params.body_size_m << 0.3762, 0.0935, 0.11;
    params.mass.total_mass_kg = 10.5;
    params.mass.body_mass_kg = 6.0;
    params.mass.com_offset_m << 0.04, 0.0, 0.0;
    params.mass.body_inertia_kg_m2_diag << 0.0337, 0.0407, 0.0613;
    params.mass.whole_robot_inertia_kg_m2_diag << 0.0792, 0.2085, 0.2265;
    params.hip_mounts_in_body[FR] << 0.1325, 0.0565, 0.0;
    params.hip_mounts_in_body[FL] << 0.1325, -0.0565, 0.0;
    params.hip_mounts_in_body[RR] << -0.1325, 0.0565, 0.0;
    params.hip_mounts_in_body[RL] << -0.1325, -0.0565, 0.0;
    params.l0 = 0.085;
    params.l1 = 0.225;
    params.l2 = 0.257;
    params.joint_limits.q0 << -2.6, 2.6;
    params.joint_limits.q1 << -6.5, 6.5;
    params.joint_limits.q2 << -2.3, 2.3;
    params.stand_targets.normal_feet_in_hip[FR] << 0.13, 0.099, -0.335;
    params.stand_targets.normal_feet_in_hip[FL] << 0.13, -0.099, -0.335;
    params.stand_targets.normal_feet_in_hip[RR] << -0.13, 0.099, -0.335;
    params.stand_targets.normal_feet_in_hip[RL] << -0.13, -0.099, -0.335;
    params.stand_targets.crouch_feet_in_hip = params.stand_targets.normal_feet_in_hip;
    ValidateRightPositiveYConvention(params);
    return params;
}

}  // namespace qr_guide
