#pragma once

#include <array>
#include <string>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

struct SafetyLimits {
  double imu_timeout_s = 0.10;
  double max_temperature_c = 80.0;
  double max_roll_pitch_rad = 0.75;
  double max_policy_age_s = 0.10;
  double joint_position_tolerance_rad = 0.005;
  int max_consecutive_io_failures = 3;
  std::array<double, kJointCount> lower_position{};
  std::array<double, kJointCount> upper_position{};
  std::array<double, kJointCount> effort_limit{};
};

struct SafetyInput {
  JointSample joints;
  ImuSample imu;
  double now_seconds = 0.0;
  double policy_age_s = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  int consecutive_io_failures = 0;
  bool communication_ok = true;
  bool physical_estop = false;
  bool software_estop = false;
  bool solver_valid = true;
  bool dynamic_mode = false;
  bool check_joint_limits = true;
};

class SafetyMonitor {
 public:
  explicit SafetyMonitor(SafetyLimits limits);

  bool Evaluate(const SafetyInput& input);
  void Reset();
  bool faulted() const { return faulted_; }
  const std::string& reason() const { return reason_; }
  const SafetyLimits& limits() const { return limits_; }

 private:
  bool Trip(const std::string& reason);

  SafetyLimits limits_;
  bool faulted_ = false;
  std::string reason_;
};

}  // namespace custom_dog_control
