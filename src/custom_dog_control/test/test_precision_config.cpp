#include "custom_dog_control/model/RobotModel.hpp"
#include "custom_dog_control/precision/PrecisionConfig.hpp"
#include "custom_dog_control/precision/PrecisionModel.hpp"
#include "custom_dog_control/precision/PrecisionPlanner.hpp"
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <unistd.h>
using namespace custom_dog_control;

TEST(PrecisionConfig, RejectsNonfiniteAndContradictoryThresholds) {
  PrecisionConfig c;
  EXPECT_NO_THROW(c.Validate());
  c.solve_timeout_ms = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(c.Validate(), std::invalid_argument);
  c = {};
  c.load_on_n = c.load_off_n;
  EXPECT_THROW(c.Validate(), std::invalid_argument);
  c = {};
  c.touchdown_timeout_s = c.confirm_dwell_s;
  EXPECT_THROW(c.Validate(), std::invalid_argument);
  c = {};
  c.early_contact_delay_s = c.swing_s;
  EXPECT_THROW(c.Validate(), std::invalid_argument);
}
TEST(PrecisionConfig, FilesAreCompleteAndTyposRejected) {
  const auto root = std::string(CUSTOM_DOG_CONTROL_SOURCE_DIR);
  EXPECT_NO_THROW(
      PrecisionConfig::Load(root + "/config/precision_control.yaml"));
  EXPECT_NO_THROW(
      RobotModelConfig::Load(root + "/config/precision_model.yaml"));
  const auto p = std::filesystem::temp_directory_path() /
                 ("qr-bad-config-" + std::to_string(getpid()) + ".yaml");
  {
    std::ofstream file(p);
    file << "control:\n  unknown_key: 1\n";
  }
  EXPECT_ANY_THROW(PrecisionConfig::Load(p.string()));
  std::filesystem::remove(p);
}
TEST(RobotModel, LoadsWithoutNmpcAndRejectsWrongSoleSize) {
  RobotModelConfig config;
  RobotModel robot(CUSTOM_DOG_CONTROL_CANONICAL_URDF, config);
  EXPECT_EQ(robot.info.stateDim, 24);
  EXPECT_EQ(robot.info.numThreeDofContacts, 4);
  config.foot_radius_m += .01;
  EXPECT_THROW(RobotModel(CUSTOM_DOG_CONTROL_CANONICAL_URDF, config),
               std::invalid_argument);
}
TEST(PrecisionPlanner, RefusesUnreadyAndUnknownSurfaceWithoutRos) {
  RobotModelConfig m;
  PrecisionConfig c;
  RobotModel robot(CUSTOM_DOG_CONTROL_CANONICAL_URDF, m);
  PrecisionPlanner planner(robot, CUSTOM_DOG_CONTROL_CANONICAL_URDF,
                           std::string(CUSTOM_DOG_CONTROL_SOURCE_DIR) +
                               "/config/nmpc/task.info",
                           m, c);
  PrecisionSnapshot state;
  PlanningRequest request;
  request.target.setZero();
  EXPECT_EQ(planner.Plan(request, state, {}, 1.).error,
            PrecisionError::STATE_NOT_READY);
  state.ready = true;
  state.now = 1.;
  state.steady_stamp = PrecisionSteadyNow();
  EXPECT_EQ(planner.Plan(request, state, {}, 1.).error,
            PrecisionError::UNKNOWN_SURFACE);
  state.steady_stamp -= .2;
  EXPECT_EQ(planner.Plan(request, state, {}, .9).error,
            PrecisionError::STATE_NOT_READY);
}

TEST(PrecisionSnapshot, FreshnessUsesMonotonicAcquisitionNotRosClockDelivery) {
  PrecisionSnapshot s;
  s.steady_stamp = 10.;
  s.now = 1234.;
  EXPECT_TRUE(SnapshotFresh(s, 10.05, .1));
  EXPECT_FALSE(SnapshotFresh(s, 10.2, .1));
  EXPECT_FALSE(SnapshotFresh(s, 9.9, .1));
  EXPECT_FALSE(SnapshotFresh(s, std::numeric_limits<double>::quiet_NaN(), .1));
}

TEST(PrecisionPlanner, ConfigurableDwellStillValidatesCompleteReturnToStance) {
  RobotModelConfig m;
  PrecisionConfig c;
  c.confirm_dwell_s = .5;
  c.Validate();
  RobotModel robot(CUSTOM_DOG_CONTROL_CANONICAL_URDF, m);
  PrecisionModel geometry(robot.pin, robot.info,
                          CUSTOM_DOG_CONTROL_CANONICAL_URDF, m, c);
  PrecisionSnapshot state;
  state.joints.valid.fill(1.);
  for (size_t f = 0; f < 4; ++f) {
    state.joints.position[3 * f + 1] = .8;
    state.joints.position[3 * f + 2] = -1.6;
  }
  state.estimate.position.z() = m.initial_height_m;
  geometry.Measure(state.joints, state.estimate, 0);
  state.estimate.position.z() += m.foot_radius_m - geometry.feet()[0].z();
  geometry.Measure(state.joints, state.estimate, 0);
  state.ref.body = state.estimate.position;
  state.ref.foot = geometry.feet();
  state.ready = true;
  state.now = 1.;
  state.steady_stamp = PrecisionSteadyNow();
  PlanningSurface ground;
  ground.known = true;
  ground.forbidden = false;
  ground.confidence = 1.;
  ground.observed_at = 1.;
  ground.normal.z = 1.;
  ground.points = {{-5, -5, 0}, {5, -5, 0}, {5, 5, 0}, {-5, 5, 0}};
  PrecisionPlanner planner(robot, CUSTOM_DOG_CONTROL_CANONICAL_URDF,
                           std::string(CUSTOM_DOG_CONTROL_SOURCE_DIR) +
                               "/config/nmpc/task.info",
                           m, c);
  const auto result =
      planner.Plan({0, 0, state.ref.foot[0]}, state, {ground}, 1.);
  EXPECT_TRUE(result.accepted) << ErrorName(result.error);
}
