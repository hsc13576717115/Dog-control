#pragma once

#include <array>
#include <cstddef>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

inline constexpr double kDegreesToRadians =
    3.14159265358979323846 / 180.0;
// 手动折腿、Gazebo 初始姿态和 START 标定共用的 hip/thigh/calf 输出侧角度。
inline constexpr std::array<double, kJointsPerLeg> kProneCalibrationPose = {
    0.0, 71.8 * kDegreesToRadians, -161.8 * kDegreesToRadians};

// 编码器电机侧 rad -> URDF 正方向的输出侧 rad，尚未扣除零点偏移。
// 以下转换要求 gear_ratio > 0、motor_direction 为 +/-1，由调用方保证。
inline double MotorPositionToUncalibratedUrdf(
    double motor_position, double gear_ratio, double motor_direction) {
  return motor_position / gear_ratio * motor_direction;
}

// 标定假设实物已摆到 calibration_pose；记录读数与已知姿态之差，
// 不驱动电机自动寻找零位。偏移始终存储在关节输出侧，单位 rad。
inline double CalibrationOffset(
    double uncalibrated_urdf_position, std::size_t joint_in_leg,
    const std::array<double, kJointsPerLeg>& calibration_pose =
        kProneCalibrationPose) {
  return uncalibrated_urdf_position - calibration_pose[joint_in_leg];
}

inline double CalibratedUrdfPosition(
    double uncalibrated_urdf_position, double calibration_offset) {
  return uncalibrated_urdf_position - calibration_offset;
}

inline double UrdfPositionToMotor(
    double urdf_position, double calibration_offset,
    double gear_ratio, double motor_direction) {
  return (urdf_position + calibration_offset) * gear_ratio * motor_direction;
}

}  // namespace custom_dog_control
