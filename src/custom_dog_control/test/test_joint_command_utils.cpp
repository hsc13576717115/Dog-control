#include <gtest/gtest.h>

#include "custom_dog_control/control/JointCommandUtils.hpp"

namespace custom_dog_control {
namespace {

TEST(JointCommand, PositionPdUsesHipThighCalfOrderOnEveryLeg) {
  std::array<double, kJointCount> pose{};
  pose.fill(0.4);
  const auto command = PositionPdCommand(pose, {10.0, 20.0, 30.0}, 2.0);
  JointSample measured;
  measured.position.fill(0.1);
  measured.velocity.fill(0.5);
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    const auto i = leg * kJointsPerLeg;
    EXPECT_NEAR(EquivalentEffort(command, measured, i), 2.0, 1e-12);
    EXPECT_NEAR(EquivalentEffort(command, measured, i + 1), 5.0, 1e-12);
    EXPECT_NEAR(EquivalentEffort(command, measured, i + 2), 8.0, 1e-12);
  }
}

TEST(JointCommand, HandoffPreservesEndpointsAndContinuousNetTorque) {
  JointSample measured;
  HybridJointCommand stand;
  HybridJointCommand wbc;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    measured.position[i] = -0.2 + 0.03 * i;
    measured.velocity[i] = 0.1 * i;
    stand.position[i] = 0.3;
    stand.kp[i] = 80.0;
    stand.kd[i] = 3.0;
    wbc.position[i] = -0.1;
    wbc.velocity[i] = 0.4;
    wbc.effort[i] = 2.0 - i;
    wbc.kp[i] = 5.0;
    wbc.kd[i] = 0.5;
  }
  for (const double alpha : {0.0, 0.1, 0.5, 0.9, 1.0}) {
    const auto blended = BlendHybridCommands(stand, wbc, measured, alpha);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double start = EquivalentEffort(stand, measured, i);
      const double end = EquivalentEffort(wbc, measured, i);
      EXPECT_NEAR(EquivalentEffort(blended, measured, i),
                  start + alpha * (end - start), 1e-12);
      if (alpha == 0.0 || alpha == 1.0) {
        const auto& endpoint = alpha == 0.0 ? stand : wbc;
        EXPECT_DOUBLE_EQ(blended.position[i], endpoint.position[i]);
        EXPECT_DOUBLE_EQ(blended.velocity[i], endpoint.velocity[i]);
        EXPECT_DOUBLE_EQ(blended.kp[i], endpoint.kp[i]);
        EXPECT_DOUBLE_EQ(blended.kd[i], endpoint.kd[i]);
        EXPECT_NEAR(blended.effort[i], endpoint.effort[i], 1e-12);
      }
    }
  }
}

}  // namespace
}  // namespace custom_dog_control
