#pragma once

#include <array>
#include <cstddef>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

inline constexpr double kDegreesToRadians =
    3.14159265358979323846 / 180.0;
// Shared nominal pose for manual folding, Gazebo startup and START calibration.
inline constexpr std::array<double, kJointsPerLeg> kProneCalibrationPose = {
    0.0, 71.8 * kDegreesToRadians, -161.8 * kDegreesToRadians};

inline double MotorPositionToUncalibratedUrdf(
    double motor_position, double gear_ratio, double motor_direction) {
  return motor_position / gear_ratio * motor_direction;
}

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
