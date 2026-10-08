#pragma once

#include <array>
#include <cstddef>

namespace custom_dog_rl {

inline constexpr std::size_t kFrameSize = 45;
inline constexpr std::size_t kHistoryFrames = 6;
inline constexpr std::size_t kObservationSize = kFrameSize * kHistoryFrames;
inline constexpr std::size_t kActionSize = 12;
inline constexpr double kPolicyPeriodS = 0.02;

using JointArray = std::array<double, kActionSize>;
using Observation = std::array<float, kObservationSize>;
using Action = std::array<float, kActionSize>;

// Policy: FL/FR/RL/RR hips, then thighs, then calves. SDK: FR/FL/RR/RL legs,
// with hip/thigh/calf inside each leg. Values are destination SDK indices.
inline constexpr std::array<std::size_t, kActionSize> kPolicyToSdk = {
    3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8};
inline constexpr JointArray kPolicyDefaultPositions = {
    0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 0.8, 0.8, -1.5, -1.5, -1.5, -1.5};
inline constexpr std::array<const char*, kActionSize> kSdkJointNames = {
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

struct SensorFrame {
  // Already calibrated URDF joint-output coordinates, rad and rad/s. The
  // hardware layer alone applies motor direction, gear ratio and zero offset.
  JointArray position_sdk{};
  JointArray velocity_sdk{};
  // Rotation from base to world; w,x,y,z. Gravity uses its inverse. This must
  // describe the robot base, after the physical IMU mounting transform.
  std::array<double, 4> orientation_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> angular_velocity_body{};
};

// Fixed deployment contract for the exported HIM policy. This class does no
// I/O, allocation, filtering or artificial observation-noise injection.
// Call observe/acceptAction at 50 Hz from one worker, not at the actuator rate.
class PolicyCore {
 public:
  void reset() noexcept;

  // Command is base-frame vx,vy (m/s), yaw rate (rad/s). History is newest
  // first. After reset, only the first frame is populated; five older frames
  // and the previous raw action are zero. Invalid input leaves state intact.
  const Observation& observe(const SensorFrame& sensors,
                             const std::array<double, 3>& command);
  void acceptAction(const Action& action);

  // Decodes raw policy output to SDK-ordered joint targets. This reproduces
  // training's affine-then-clip convention; [-100,100] is NOT a physical
  // joint limit. The hardware controller must impose its own safe envelope.
  JointArray decode(const Action& action) const;
  static const JointArray& defaultPositionsSdk() noexcept;

 private:
  Observation history_{};
  Action previous_action_{};
};

}  // namespace custom_dog_rl
