#pragma once
#include "custom_dog_control/model/RobotModelConfig.hpp"
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
namespace custom_dog_control {
// Owns model/data. Consumers copy the model to keep mutable Pinocchio caches
// private.
class RobotModel {
public:
  RobotModel(const std::string &urdf, const RobotModelConfig &config);
  ocs2::PinocchioInterface pin;
  ocs2::CentroidalModelInfo info;
};
} // namespace custom_dog_control
