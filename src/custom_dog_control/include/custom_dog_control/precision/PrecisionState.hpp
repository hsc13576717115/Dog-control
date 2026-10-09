#pragma once
#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"
#include "custom_dog_control/precision/PrecisionTypes.hpp"
#include <chrono>
namespace custom_dog_control {
struct PrecisionSnapshot {
  JointSample joints;
  EstimatedState estimate;
  WholeBodyReference ref;
  std::array<ContactEstimate, 4> contact{};
  StepPhase phase = StepPhase::STANCE;
  uint64_t id = 0;
  double now = 0, solve_ms = 0, residual = 0;
  double steady_stamp =
      0; // Acquisition/compute freshness independent of ROS clock delivery.
  bool ready = false;
  PrecisionError error = PrecisionError::NONE;
};
inline double PrecisionSteadyNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
inline bool SnapshotFresh(const PrecisionSnapshot &s, double now,
                          double timeout) {
  const double age = now - s.steady_stamp;
  return std::isfinite(age) && s.steady_stamp > 0 && age >= 0 && age <= timeout;
}
struct PrecisionCommand {
  PrecisionStep step;
  bool valid = false;
};
inline std::size_t ContactMode(const std::array<bool, 4> &flags) {
  std::size_t mode = 0;
  for (size_t f = 0; f < 4; ++f)
    if (flags[f])
      mode |= (1U << (3 - f));
  return mode;
}
} // namespace custom_dog_control
