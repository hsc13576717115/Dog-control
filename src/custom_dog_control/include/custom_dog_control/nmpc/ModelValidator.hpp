#pragma once

#include <array>
#include <string>
#include <vector>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

struct ModelValidationResult {
  bool ok = false;
  double total_mass_kg = 0.0;
  // Hardware order is FR, FL, RR, RL. Values are model q/v slots after the
  // six floating-base coordinates and may differ from the hardware index.
  std::array<int, kJointCount> model_joint_slots{};
  std::vector<std::string> errors;

  std::string Summary() const;
};

class ModelValidator {
 public:
  static ModelValidationResult Validate(
      ocs2::PinocchioInterface& interface,
      const ocs2::CentroidalModelInfo& info,
      double expected_mass_kg = 13.84916,
      double mass_tolerance_kg = 1e-5);
};

}  // namespace custom_dog_control
