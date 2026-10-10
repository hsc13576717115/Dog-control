#pragma once
#include "custom_dog_control/control/ContactTypes.hpp"
#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/precision/PrecisionError.hpp"
#include <Eigen/Core>
#include <array>
namespace custom_dog_control {
struct WholeBodyReference {
  WholeBodyReference() {
    for (auto *a : {&foot, &velocity, &acceleration})
      for (auto &v : *a)
        v.setZero();
  }
  int probe_foot = -1;
  double probe_force = 0.;
  Eigen::Vector3d body = Eigen::Vector3d::Zero(),
                  body_velocity = Eigen::Vector3d::Zero(),
                  body_acceleration = Eigen::Vector3d::Zero();
  // ZYX coordinate derivatives, not world-frame angular velocity.
  Eigen::Vector3d euler = Eigen::Vector3d::Zero(),
                  euler_velocity = Eigen::Vector3d::Zero(),
                  euler_acceleration = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, 4> foot{}, velocity{}, acceleration{};
  std::array<bool, 4> contact{true, true, true, true};
};
struct ReferenceKnot {
  double time = 0.;
  Eigen::Vector3d body = Eigen::Vector3d::Zero(),
                  euler = Eigen::Vector3d::Zero(),
                  foot = Eigen::Vector3d::Zero();
};
// Fixed capacity bounds the command copy and interpolation work in control.
// Contact transitions remain executor events; knots cannot declare touchdown.
struct CoordinatedReference {
  static constexpr size_t capacity = 192;
  CoordinatedReference();
  CoordinatedReference(const CoordinatedReference &);
  CoordinatedReference &operator=(const CoordinatedReference &);
  std::array<ReferenceKnot, capacity> knots;
  size_t size = 0, lift_index = 0;
};
struct PrecisionStep {
  uint64_t id = 0;
  size_t foot = 0;
  Eigen::Vector3d target = Eigen::Vector3d::Zero(),
                  body = Eigen::Vector3d::Zero();
  // Optional final body reference; absent preserves the M1 restore behavior.
  bool has_body_finish = false;
  Eigen::Vector3d body_finish = Eigen::Vector3d::Zero();
  double shift = 2., swing = 1.5, clearance = .06, timeout = .8;
  CoordinatedReference trajectory;
};
enum class StepPhase : uint8_t {
  STANCE = 0,
  SHIFT = 1,
  SWING = 2,
  CONFIRM = 3,
  RESTORE = 4,
  HOLD = 5,
  DONE = 6
};
inline const char *PhaseName(StepPhase p) {
  switch (p) {
  case StepPhase::STANCE:
    return "PRECISION_STANCE";
  case StepPhase::SHIFT:
    return "SHIFT";
  case StepPhase::SWING:
    return "SWING";
  case StepPhase::CONFIRM:
    return "CONTACT_CONFIRM";
  case StepPhase::RESTORE:
    return "RESTORE";
  case StepPhase::HOLD:
    return "CONTROLLED_HOLD";
  case StepPhase::DONE:
    return "DONE";
  }
  return "UNKNOWN";
}
} // namespace custom_dog_control
