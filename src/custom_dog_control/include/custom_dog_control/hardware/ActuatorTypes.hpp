#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

struct DriveParameters {
  std::array<std::string, kLegCount> serial_ports;
  double base_gear_ratio = 6.33;
  double calf_total_gear_ratio = 12.66;
  bool use_parallel_leg_io = true;
  std::array<double, kJointCount> motor_directions{
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, kJointsPerLeg> calibration_pose{
      0.0, 1.2531464029319286, -2.8239427297268254};
};

enum class ActuatorControlMode : std::uint32_t {
  DISABLE = 0,
  COMPOUND = 1,
};

struct ActuatorCommand {
  std::uint32_t mode = static_cast<std::uint32_t>(ActuatorControlMode::DISABLE);
  float position = 0.0F;
  float velocity = 0.0F;
  float effort = 0.0F;
  float kp = 0.0F;
  float kd = 0.0F;
};

struct LowLevelCommand {
  std::array<ActuatorCommand, kJointCount> motors{};
};

struct ActuatorState {
  float position = 0.0F;
  float velocity = 0.0F;
  float effort = 0.0F;
  int temperature = 0;
  int fault = 0;
};

struct LowLevelState {
  std::array<ActuatorState, kJointCount> motors{};
};

}  // namespace custom_dog_control
