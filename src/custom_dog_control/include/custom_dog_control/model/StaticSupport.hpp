#pragma once
#include "custom_dog_control/model/RobotModel.hpp"
#include <array>
#include <limits>
namespace custom_dog_control {
struct StaticSupportResult {
  bool feasible = false;
  bool infeasible = false;
  int solver_return_code = -1;
  Eigen::Matrix<double, 12, 1> forces = Eigen::Matrix<double, 12, 1>::Zero();
  double residual = std::numeric_limits<double>::infinity();
};
// Horizontal supports; sole_radius shifts force application from the foot
// centre to the sphere contact point (zero preserves the legacy point model).
// Offline necessary feasibility check. All accelerations are constrained to
// zero; no WBC tracking weights or torque slack can hide an imbalance. Does
// not certify geometry, friction identification, transition dynamics or reach.
StaticSupportResult CheckStaticSupport(const RobotModel &,
                                       const Eigen::VectorXd &q,
                                       const std::array<bool, 4> &contacts,
                                       double friction,
                                       double sole_radius = 0.);
} // namespace custom_dog_control
