#pragma once
#include "custom_dog_control/control/ControlTypes.hpp"
#include <Eigen/Core>
#include <array>
namespace custom_dog_control {
// These are estimates from joint effort/kinematics, not measured sole forces.
struct ContactEstimate {
  bool valid = false, loaded = false, slipping = false;
  double stamp = 0, probability = 0;
  Eigen::Vector3d estimated_force = Eigen::Vector3d::Zero();
};
struct ContactSupport {
  ContactSupport() {
    for (auto *array : {&anchor_position, &center_velocity})
      for (auto &p : *array)
        p.setZero();
  }
  std::array<bool, 4> contact{};
  std::array<bool, 4> anchor_valid{};
  std::array<Eigen::Vector3d, 4> anchor_position{};
  // Expected centre velocity for an explicitly configured rolling spherical
  // sole. Zero preserves the point-foot model; never a measured ground
  // velocity.
  std::array<Eigen::Vector3d, 4> center_velocity{};
  std::array<bool, 4> height_valid{};
  std::array<double, 4> foot_center_height{};
};
} // namespace custom_dog_control
