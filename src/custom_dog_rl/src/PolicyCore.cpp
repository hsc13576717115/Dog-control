#include "custom_dog_rl/PolicyCore.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace custom_dog_rl {
namespace {

template <typename Values>
void requireFinite(const Values& values, const char* description) {
  for (const auto value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(description);
    }
  }
}

float observationValue(double value, double scale = 1.0) {
  // The Isaac observation manager clips each term before applying its scale.
  return static_cast<float>(std::clamp(value, -100.0, 100.0) * scale);
}

std::array<double, 3> projectedGravity(const std::array<double, 4>& quaternion) {
  const double norm = std::hypot(std::hypot(quaternion[0], quaternion[1]),
                                 std::hypot(quaternion[2], quaternion[3]));
  if (!std::isfinite(norm) || norm < 1e-12) {
    throw std::invalid_argument("orientation quaternion has invalid norm");
  }
  const double w = quaternion[0] / norm;
  const double x = quaternion[1] / norm;
  const double y = quaternion[2] / norm;
  const double z = quaternion[3] / norm;
  // R(q)^T * [0,0,-1]. This is a unit direction, not raw IMU acceleration.
  return {2.0 * (w * y - x * z), -2.0 * (y * z + w * x),
          2.0 * (x * x + y * y) - 1.0};
}

}  // namespace

void PolicyCore::reset() noexcept {
  history_.fill(0.0F);
  previous_action_.fill(0.0F);
}

const Observation& PolicyCore::observe(const SensorFrame& sensors,
                                       const std::array<double, 3>& command) {
  requireFinite(command, "nonfinite velocity command");
  requireFinite(sensors.position_sdk, "nonfinite joint position");
  requireFinite(sensors.velocity_sdk, "nonfinite joint velocity");
  requireFinite(sensors.orientation_wxyz, "nonfinite orientation");
  requireFinite(sensors.angular_velocity_body, "nonfinite angular velocity");
  const auto gravity = projectedGravity(sensors.orientation_wxyz);

  std::array<float, kFrameSize> frame{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    frame[axis] = observationValue(command[axis]);
    frame[3 + axis] = observationValue(sensors.angular_velocity_body[axis], 0.25);
    frame[6 + axis] = observationValue(gravity[axis]);
  }
  for (std::size_t joint = 0; joint < kActionSize; ++joint) {
    const auto sdk_joint = kPolicyToSdk[joint];
    frame[9 + joint] = observationValue(
        sensors.position_sdk[sdk_joint] - kPolicyDefaultPositions[joint]);
    frame[21 + joint] = observationValue(sensors.velocity_sdk[sdk_joint], 0.05);
    frame[33 + joint] = observationValue(previous_action_[joint]);
  }

  // Build and validate the complete new frame before changing persistent state.
  std::move_backward(history_.begin(), history_.end() - kFrameSize, history_.end());
  std::copy(frame.begin(), frame.end(), history_.begin());
  return history_;
}

void PolicyCore::acceptAction(const Action& action) {
  requireFinite(action, "nonfinite raw policy action");
  previous_action_ = action;
}

JointArray PolicyCore::decode(const Action& action) const {
  requireFinite(action, "nonfinite raw policy action");
  JointArray target_sdk{};
  for (std::size_t joint = 0; joint < kActionSize; ++joint) {
    // Preserve FP32 arithmetic used by Isaac JointPositionAction.
    const float target = static_cast<float>(kPolicyDefaultPositions[joint]) +
                         0.25F * action[joint];
    target_sdk[kPolicyToSdk[joint]] = std::clamp(target, -100.0F, 100.0F);
  }
  return target_sdk;
}

const JointArray& PolicyCore::defaultPositionsSdk() noexcept {
  static constexpr JointArray positions = [] {
    JointArray result{};
    for (std::size_t joint = 0; joint < kActionSize; ++joint) {
      result[kPolicyToSdk[joint]] = kPolicyDefaultPositions[joint];
    }
    return result;
  }();
  return positions;
}

}  // namespace custom_dog_rl
