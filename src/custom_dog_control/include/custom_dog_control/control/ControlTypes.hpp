#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace custom_dog_control {

constexpr std::size_t kLegCount = 4;
constexpr std::size_t kJointsPerLeg = 3;
constexpr std::size_t kJointCount = kLegCount * kJointsPerLeg;
constexpr std::size_t kCentroidalStateDim = 24;
constexpr std::size_t kCentroidalInputDim = 24;
constexpr std::size_t kStanceMode = 15U;
constexpr std::size_t kTrotFrRlMode = 9U;
constexpr std::size_t kTrotFlRrMode = 6U;
// A single fixed contact schedule avoids changing the OCS2 mode sequence
// while the solver is running. Zero velocity is handled as full stance.
constexpr double kTrotPeriodSeconds = 0.25;
constexpr double kTrotHalfPeriodSeconds = 0.125;

inline constexpr std::array<std::string_view, kJointCount> kJointNames = {
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

inline constexpr std::array<std::string_view, kLegCount> kFootFrameNames = {
    "FR_foot", "FL_foot", "RR_foot", "RL_foot"};

enum class OperatingMode : std::uint8_t {
  PASSIVE = 0,
  CALIBRATION,
  STAND_UP,
  MPC_STANCE,
  MPC_TROT,
  FAULT,
};

inline constexpr std::string_view ToString(OperatingMode mode) {
  switch (mode) {
    case OperatingMode::PASSIVE:
      return "PASSIVE";
    case OperatingMode::CALIBRATION:
      return "CALIBRATION";
    case OperatingMode::STAND_UP:
      return "STAND_UP";
    case OperatingMode::MPC_STANCE:
      return "MPC_STANCE";
    case OperatingMode::MPC_TROT:
      return "MPC_TROT";
    case OperatingMode::FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

struct VelocityCommand {
  double vx = 0.0;
  double vy = 0.0;
  double yaw = 0.0;
  double stamp_seconds = 0.0;
  bool active = false;
};

struct VelocityLimits {
  double vx = 0.30;
  double vy = 0.10;
  double yaw = 0.30;
  double acceleration_xy = 0.60;
  double acceleration_yaw = 0.80;
};

inline VelocityCommand ClampVelocity(const VelocityCommand& input, const VelocityLimits& limits) {
  VelocityCommand output = input;
  output.vx = std::clamp(output.vx, -limits.vx, limits.vx);
  output.vy = std::clamp(output.vy, -limits.vy, limits.vy);
  output.yaw = std::clamp(output.yaw, -limits.yaw, limits.yaw);
  return output;
}

inline double LegacyJoyLateralToRep103(double lateral, bool legacy_y_right) {
  return legacy_y_right ? -lateral : lateral;
}

inline std::size_t TrotModeAtTime(double time_seconds) {
  double phase = std::fmod(time_seconds, kTrotPeriodSeconds);
  if (phase < 0.0) {
    phase += kTrotPeriodSeconds;
  }
  if (phase < kTrotHalfPeriodSeconds) {
    return kTrotFrRlMode;
  }
  return kTrotFlRrMode;
}

inline bool IsTrotMode(std::size_t mode) {
  return mode == kTrotFrRlMode || mode == kTrotFlRrMode;
}

inline bool ExceedsTrotEntryThreshold(const VelocityCommand& command) {
  return std::abs(command.vx) > 0.04 || std::abs(command.vy) > 0.025 ||
         std::abs(command.yaw) > 0.15;
}

inline bool IsBelowTrotExitThreshold(const VelocityCommand& command) {
  return std::abs(command.vx) < 0.02 && std::abs(command.vy) < 0.015 &&
         std::abs(command.yaw) < 0.08;
}

inline double Slew(double current, double target, double max_rate, double dt) {
  const double delta = std::clamp(target - current, -max_rate * dt, max_rate * dt);
  return current + delta;
}

inline VelocityCommand SlewVelocity(
    const VelocityCommand& current, const VelocityCommand& target,
    const VelocityLimits& limits, double dt) {
  VelocityCommand output = target;
  output.vx = Slew(current.vx, target.vx, limits.acceleration_xy, dt);
  output.vy = Slew(current.vy, target.vy, limits.acceleration_xy, dt);
  output.yaw = Slew(current.yaw, target.yaw, limits.acceleration_yaw, dt);
  return output;
}

struct JointSample {
  std::array<double, kJointCount> position{};
  std::array<double, kJointCount> velocity{};
  std::array<double, kJointCount> effort{};
  std::array<double, kJointCount> temperature{};
  std::array<double, kJointCount> valid{};
};

struct HybridJointCommand {
  std::array<double, kJointCount> position{};
  std::array<double, kJointCount> velocity{};
  std::array<double, kJointCount> effort{};
  std::array<double, kJointCount> kp{};
  std::array<double, kJointCount> kd{};
};

struct ImuSample {
  std::array<double, 4> orientation_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
  double stamp_seconds = 0.0;
  bool valid = false;
};

struct PolicySample {
  std::array<double, kCentroidalStateDim> state{};
  std::array<double, kCentroidalInputDim> input{};
  std::uint64_t sequence = 0;
  std::size_t mode = kStanceMode;
  double observation_time = 0.0;
  double generated_at_seconds = 0.0;
  double solve_time_ms = 0.0;
  bool valid = false;
};

inline constexpr std::array<bool, kLegCount> ContactFlags(std::size_t mode) {
  return {
      static_cast<bool>(mode & 0x8U),
      static_cast<bool>(mode & 0x4U),
      static_cast<bool>(mode & 0x2U),
      static_cast<bool>(mode & 0x1U)};
}

}  // namespace custom_dog_control
