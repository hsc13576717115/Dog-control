#include <array>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>

#include "custom_dog_control/nmpc/ModelValidator.hpp"

namespace {

TEST(ModelValidator, CanonicalModelAndHardwareOrderAreValid) {
  const std::vector<std::string> joints = {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
  const std::vector<std::string> feet = {
      "FR_foot", "FL_foot", "RR_foot", "RL_foot"};
  auto pinocchio_interface =
      ocs2::centroidal_model::createPinocchioInterface(
          CUSTOM_DOG_CONTROL_CANONICAL_URDF, joints);
  const auto model_info = ocs2::centroidal_model::createCentroidalModelInfo(
      pinocchio_interface,
      ocs2::CentroidalModelType::FullCentroidalDynamics,
      ocs2::vector_t::Zero(joints.size()), feet, {});

  auto result = custom_dog_control::ModelValidator::Validate(
      pinocchio_interface, model_info);

  EXPECT_TRUE(result.ok) << result.Summary();
  EXPECT_NEAR(result.total_mass_kg, 13.84916, 1e-5);
  EXPECT_EQ(
      result.model_joint_slots,
      (std::array<int, 12>{3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8}));
}

}  // namespace
