#include "custom_dog_control/model/RobotModel.hpp"
#include "custom_dog_control/model/ModelValidator.hpp"
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <stdexcept>
#include <urdf/model.h>
namespace custom_dog_control {
namespace {
std::vector<std::string> Joints() {
  std::vector<std::string> names;
  for (auto name : kJointNames)
    names.emplace_back(name);
  return names;
}
std::vector<std::string> Feet() {
  std::vector<std::string> names;
  for (auto name : kFootFrameNames)
    names.emplace_back(name);
  return names;
}
} // namespace
RobotModel::RobotModel(const std::string &urdf, const RobotModelConfig &config)
    : pin(ocs2::centroidal_model::createPinocchioInterface(urdf, Joints())),
      info(ocs2::centroidal_model::createCentroidalModelInfo(
          pin, ocs2::CentroidalModelType::FullCentroidalDynamics,
          Eigen::VectorXd::Zero(12), Feet(), {})) {
  config.Validate();
  const auto result = ModelValidator::Validate(
      pin, info, config.expected_mass_kg, config.mass_tolerance_kg);
  if (!result.ok)
    throw std::invalid_argument(result.Summary());
  urdf::Model description;
  if (!description.initFile(urdf))
    throw std::invalid_argument("invalid URDF");
  for (auto name : kFootFrameNames) {
    const auto link = description.getLink(std::string(name));
    const auto sphere =
        link && link->collision
            ? std::dynamic_pointer_cast<urdf::Sphere>(link->collision->geometry)
            : nullptr;
    if (!sphere || std::abs(sphere->radius - config.foot_radius_m) > 1e-6)
      throw std::invalid_argument(
          "configured foot radius differs from spherical URDF sole");
  }
}
} // namespace custom_dog_control
