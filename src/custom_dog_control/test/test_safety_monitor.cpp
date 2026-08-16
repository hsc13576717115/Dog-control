#include <gtest/gtest.h>

#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"
#include "custom_dog_control/safety/SafetyMonitor.hpp"

namespace custom_dog_control {
namespace {

SafetyLimits Limits() {
  SafetyLimits limits;
  limits.lower_position.fill(-2.0);
  limits.upper_position.fill(2.0);
  limits.effort_limit.fill(50.0);
  return limits;
}

SafetyInput ValidInput() {
  SafetyInput input;
  input.now_seconds = 10.0;
  input.imu.valid = true;
  input.imu.stamp_seconds = 10.0;
  input.joints.valid.fill(1.0);
  input.solver_valid = true;
  return input;
}

TEST(SafetyMonitor, AcceptsNominalState) {
  SafetyMonitor monitor(Limits());
  EXPECT_TRUE(monitor.Evaluate(ValidInput()));
  EXPECT_FALSE(monitor.faulted());
}

TEST(SafetyMonitor, FaultIsLatchedUntilReset) {
  SafetyMonitor monitor(Limits());
  auto input = ValidInput();
  input.physical_estop = true;
  EXPECT_FALSE(monitor.Evaluate(input));
  EXPECT_EQ(monitor.reason(), "physical emergency stop");

  input.physical_estop = false;
  EXPECT_FALSE(monitor.Evaluate(input));
  monitor.Reset();
  EXPECT_TRUE(monitor.Evaluate(input));
}

TEST(SafetyMonitor, RejectsStaleDynamicPolicy) {
  SafetyMonitor monitor(Limits());
  auto input = ValidInput();
  input.dynamic_mode = true;
  input.policy_age_s = 0.11;
  EXPECT_FALSE(monitor.Evaluate(input));
  EXPECT_EQ(monitor.reason(), "NMPC policy stale or invalid");
}

TEST(SafetyMonitor, RejectsImuMotorAndLimitFailures) {
  {
    SafetyMonitor monitor(Limits());
    auto input = ValidInput();
    input.imu.stamp_seconds = 9.8;
    EXPECT_FALSE(monitor.Evaluate(input));
    EXPECT_EQ(monitor.reason(), "IMU timeout");
  }
  {
    SafetyMonitor monitor(Limits());
    auto input = ValidInput();
    input.joints.valid[4] = 0.0;
    EXPECT_FALSE(monitor.Evaluate(input));
    EXPECT_EQ(monitor.reason(), "invalid motor state");
  }
  {
    SafetyMonitor monitor(Limits());
    auto input = ValidInput();
    input.joints.position[7] = 2.1;
    EXPECT_FALSE(monitor.Evaluate(input));
    EXPECT_EQ(monitor.reason().find("joint position limit: index=7"), 0U);
  }
}

TEST(SafetyMonitor, JointLimitToleranceRejectsOnlyMaterialViolations) {
  const auto limits = Limits();
  {
    SafetyMonitor monitor(limits);
    auto input = ValidInput();
    input.joints.position[0] = 2.0 + 0.5 * limits.joint_position_tolerance_rad;
    EXPECT_TRUE(monitor.Evaluate(input));
  }
  {
    SafetyMonitor monitor(limits);
    auto input = ValidInput();
    input.joints.position[0] = 2.0 + 2.0 * limits.joint_position_tolerance_rad;
    EXPECT_FALSE(monitor.Evaluate(input));
    EXPECT_EQ(monitor.reason().find("joint position limit: index=0"), 0U);
  }
}

TEST(StateEstimator, NegativeYawDoesNotBecomeFlippedRollAndPitch) {
  constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
  const double yaw = -170.0 * kDegreesToRadians;
  const double pitch = 1.0 * kDegreesToRadians;
  const double roll = -2.0 * kDegreesToRadians;
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();

  const Eigen::Vector3d euler_zyx = EulerZyxFromRotation(rotation);
  EXPECT_NEAR(euler_zyx.x(), yaw, 1e-12);
  EXPECT_NEAR(euler_zyx.y(), pitch, 1e-12);
  EXPECT_NEAR(euler_zyx.z(), roll, 1e-12);
}

}  // namespace
}  // namespace custom_dog_control
