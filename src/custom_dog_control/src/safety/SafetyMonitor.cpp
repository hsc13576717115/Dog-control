#include "custom_dog_control/safety/SafetyMonitor.hpp"

#include <cmath>
#include <sstream>
#include <utility>

namespace custom_dog_control {

SafetyMonitor::SafetyMonitor(SafetyLimits limits) : limits_(std::move(limits)) {}

bool SafetyMonitor::Trip(const std::string& reason) {
  faulted_ = true;
  reason_ = reason;
  return false;
}

bool SafetyMonitor::Evaluate(const SafetyInput& input) {
  if (faulted_) {
    return false;
  }
  if (input.physical_estop) {
    return Trip("physical emergency stop");
  }
  if (input.software_estop) {
    return Trip("software emergency stop");
  }
  if (!input.communication_ok ||
      input.consecutive_io_failures >= limits_.max_consecutive_io_failures) {
    return Trip("motor communication failure");
  }
  if (!input.imu.valid || input.now_seconds - input.imu.stamp_seconds > limits_.imu_timeout_s) {
    return Trip("IMU timeout");
  }
  if (std::abs(input.roll) > limits_.max_roll_pitch_rad ||
      std::abs(input.pitch) > limits_.max_roll_pitch_rad) {
    return Trip("base attitude limit");
  }
  if (input.dynamic_mode &&
      (!input.solver_valid || input.policy_age_s > limits_.max_policy_age_s)) {
    return Trip("NMPC policy stale or invalid");
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(input.joints.position[i]) ||
        !std::isfinite(input.joints.velocity[i]) ||
        !std::isfinite(input.joints.effort[i])) {
      return Trip("non-finite joint state");
    }
    if (input.joints.valid[i] < 0.5) {
      return Trip("invalid motor state");
    }
    if (input.joints.temperature[i] > limits_.max_temperature_c) {
      return Trip("motor over-temperature");
    }
    if (input.check_joint_limits &&
        (input.joints.position[i] <
             limits_.lower_position[i] - limits_.joint_position_tolerance_rad ||
         input.joints.position[i] >
             limits_.upper_position[i] + limits_.joint_position_tolerance_rad)) {
      std::ostringstream reason;
      reason << "joint position limit: index=" << i
             << " position=" << input.joints.position[i]
             << " lower=" << limits_.lower_position[i]
             << " upper=" << limits_.upper_position[i];
      return Trip(reason.str());
    }
    if (std::abs(input.joints.effort[i]) > limits_.effort_limit[i] * 1.10) {
      return Trip("joint effort limit");
    }
  }
  return true;
}

void SafetyMonitor::Reset() {
  faulted_ = false;
  reason_.clear();
}

}  // namespace custom_dog_control
