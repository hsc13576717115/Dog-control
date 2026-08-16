#include <gtest/gtest.h>

#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/control/TimingStatistics.hpp"
#include "custom_dog_control/hardware/ActuatorTypes.hpp"
#include "custom_dog_control/hardware/Calibration.hpp"

namespace custom_dog_control {
namespace {

TEST(ControlContract, JointAndFootOrderIsFixed) {
  EXPECT_EQ(kJointNames.front(), "FR_hip_joint");
  EXPECT_EQ(kJointNames[3], "FL_hip_joint");
  EXPECT_EQ(kJointNames[6], "RR_hip_joint");
  EXPECT_EQ(kJointNames[9], "RL_hip_joint");
  EXPECT_EQ(kFootFrameNames[0], "FR_foot");
  EXPECT_EQ(kFootFrameNames[3], "RL_foot");
}

TEST(ControlContract, DiagonalTrotUsesFixedThreeHundredMillisecondCycle) {
  EXPECT_EQ(TrotModeAtTime(0.00), kTrotFrRlMode);
  EXPECT_EQ(TrotModeAtTime(0.149), kTrotFrRlMode);
  EXPECT_EQ(TrotModeAtTime(0.150), kTrotFlRrMode);
  EXPECT_EQ(TrotModeAtTime(0.299), kTrotFlRrMode);
  EXPECT_EQ(TrotModeAtTime(0.300), kTrotFrRlMode);

  EXPECT_EQ(ContactFlags(kTrotFrRlMode),
            (std::array<bool, 4>{true, false, false, true}));
  EXPECT_EQ(ContactFlags(kTrotFlRrMode),
            (std::array<bool, 4>{false, true, true, false}));
}

TEST(ControlContract, TrotCommandThresholdsHaveHysteresis) {
  VelocityCommand command;
  EXPECT_FALSE(ExceedsTrotEntryThreshold(command));
  EXPECT_TRUE(IsBelowTrotExitThreshold(command));

  command.vx = 0.03;
  EXPECT_FALSE(ExceedsTrotEntryThreshold(command));
  EXPECT_FALSE(IsBelowTrotExitThreshold(command));

  command.vx = 0.05;
  EXPECT_TRUE(ExceedsTrotEntryThreshold(command));
  EXPECT_FALSE(IsBelowTrotExitThreshold(command));
}

TEST(ControlContract, Rep103ConversionOccursOnce) {
  EXPECT_DOUBLE_EQ(LegacyJoyLateralToRep103(0.1, true), -0.1);
  EXPECT_DOUBLE_EQ(LegacyJoyLateralToRep103(0.1, false), 0.1);
}

TEST(ControlContract, StartCalibrationAlignsDirectlyToUrdfPronePose) {
  EXPECT_NEAR(kProneCalibrationPose[0], 0.0, 1e-12);
  EXPECT_NEAR(kProneCalibrationPose[1], 71.8 * kDegreesToRadians, 1e-12);
  EXPECT_NEAR(kProneCalibrationPose[2], -161.8 * kDegreesToRadians, 1e-12);

  constexpr double kGearRatio = 6.33;
  constexpr double kDirection = -1.0;
  constexpr double kRawMotorPosition = -9.5;
  const double uncalibrated = MotorPositionToUncalibratedUrdf(
      kRawMotorPosition, kGearRatio, kDirection);
  const double offset = CalibrationOffset(uncalibrated, 1);
  EXPECT_NEAR(
      CalibratedUrdfPosition(uncalibrated, offset),
      kProneCalibrationPose[1], 1e-12);
  EXPECT_NEAR(
      UrdfPositionToMotor(
          kProneCalibrationPose[1], offset, kGearRatio, kDirection),
      kRawMotorPosition, 1e-12);
}

TEST(ControlContract, RealHardwareUsesSharedFoldedCalibrationPose) {
  const DriveParameters parameters;
  EXPECT_NEAR(parameters.calibration_pose[0], 0.0, 1e-12);
  EXPECT_NEAR(
      parameters.calibration_pose[1], 71.8 * kDegreesToRadians, 1e-12);
  EXPECT_NEAR(
      parameters.calibration_pose[2], -161.8 * kDegreesToRadians, 1e-12);
}

TEST(ControlContract, VelocityIsClampedAndSlewLimited) {
  VelocityLimits limits;
  VelocityCommand requested;
  requested.vx = 1.0;
  requested.vy = -1.0;
  requested.yaw = 1.0;
  const auto clamped = ClampVelocity(requested, limits);
  EXPECT_DOUBLE_EQ(clamped.vx, 0.30);
  EXPECT_DOUBLE_EQ(clamped.vy, -0.10);
  EXPECT_DOUBLE_EQ(clamped.yaw, 0.30);

  const auto first = SlewVelocity({}, clamped, limits, 0.1);
  EXPECT_NEAR(first.vx, 0.06, 1e-12);
  EXPECT_NEAR(first.vy, -0.06, 1e-12);
  EXPECT_NEAR(first.yaw, 0.08, 1e-12);
}

TEST(ControlContract, TimingWindowIsBounded) {
  TimingWindow<4> timing;
  timing.Add(1.0);
  timing.Add(2.0);
  timing.Add(3.0);
  timing.Add(4.0);
  timing.Add(5.0);
  const auto snapshot = timing.Snapshot();
  EXPECT_EQ(snapshot.samples, 4U);
  EXPECT_DOUBLE_EQ(snapshot.mean, 3.5);
  EXPECT_DOUBLE_EQ(snapshot.p99, 5.0);
  EXPECT_DOUBLE_EQ(snapshot.maximum, 5.0);
}

}  // namespace
}  // namespace custom_dog_control
