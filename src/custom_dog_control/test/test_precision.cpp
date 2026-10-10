#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"
#include "custom_dog_control/precision/ContactObserver.hpp"
#include "custom_dog_control/precision/FootstepExecutor.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <ocs2_centroidal_model/FactoryFunctions.h>
using namespace custom_dog_control;
TEST(ContactObserver, StillnessDoesNotProveLoad) {
  ContactObserver o;
  ContactEstimate c;
  for (int k = 1; k < 200; ++k)
    c = o.Update(k * .004, {0, 0, 0}, {0, 0, 0}, true);
  EXPECT_FALSE(c.loaded);
  for (int k = 200; k < 400; ++k)
    c = o.Update(k * .004, {0, 0, 30}, {0, 0, 0}, true);
  EXPECT_TRUE(c.loaded);
  EXPECT_FALSE(o.Update(2., {0, 0, 30}, {0, 0, 0}, false).valid);
}
TEST(ContactObserver, ImpactMustPersistAndStaleSamplesDoNotConfirm) {
  ContactObserver o;
  EXPECT_FALSE(o.Update(.004, {0, 0, 100}, {0, 0, 0}, true).loaded);
  EXPECT_FALSE(o.Update(.004, {0, 0, 100}, {0, 0, 0}, true).valid);
  EXPECT_FALSE(o.Update(.008, {0, 0, 0}, {0, 0, 0}, true).loaded);
}
static WholeBodyReference Stance() {
  WholeBodyReference r;
  r.body = {0, 0, .29};
  r.foot = {Eigen::Vector3d(.2, -.15, .026), Eigen::Vector3d(.2, .15, .026),
            Eigen::Vector3d(-.2, -.15, .026), Eigen::Vector3d(-.2, .15, .026)};
  return r;
}
TEST(Executor, MissedTouchdownNeverCompletes) {
  FootstepExecutor ex;
  auto r = Stance();
  ex.Reset(r);
  PrecisionStep s;
  s.foot = 0;
  s.target = r.foot[0] + Eigen::Vector3d(.02, 0, 0);
  s.body = r.body;
  s.shift = .2;
  s.swing = .4;
  s.timeout = .2;
  ex.Start(s, 0);
  std::array<ContactEstimate, 4> c;
  for (auto &v : c) {
    v.valid = v.loaded = true;
  }
  c[0].loaded = false;
  for (double t = 0; t < 1.2; t += .01)
    ex.Update(t, c, ex.reference().foot);
  EXPECT_EQ(ex.phase(), StepPhase::HOLD);
  EXPECT_STREQ(ex.error(), "touchdown_timeout");
}
TEST(Executor, SideContactDoesNotComplete) {
  FootstepExecutor ex;
  auto r = Stance();
  ex.Reset(r);
  PrecisionStep s;
  s.target = r.foot[0];
  s.body = r.body;
  s.shift = .2;
  s.swing = 1.;
  ex.Start(s, 0);
  std::array<ContactEstimate, 4> c;
  for (auto &v : c) {
    v.valid = v.loaded = true;
  }
  for (double t = 0; t < .8; t += .01)
    ex.Update(t, c, r.foot);
  EXPECT_EQ(ex.phase(), StepPhase::HOLD);
  EXPECT_STREQ(ex.error(), "early_contact");
}
TEST(Estimator, UnknownHeightDoesNotMeanGroundZero) {
  std::vector<std::string> j, f;
  for (auto n : kJointNames)
    j.emplace_back(n);
  for (auto n : kFootFrameNames)
    f.emplace_back(n);
  auto pin = ocs2::centroidal_model::createPinocchioInterface(
      CUSTOM_DOG_CONTROL_CANONICAL_URDF, j);
  auto info = ocs2::centroidal_model::createCentroidalModelInfo(
      pin, ocs2::CentroidalModelType::FullCentroidalDynamics,
      Eigen::VectorXd::Zero(12), f, {});
  KinematicStateEstimator flat(pin, info), raised(pin, info);
  flat.Reset(.29);
  raised.Reset(.49);
  JointSample joints;
  joints.valid.fill(1.);
  for (size_t k = 0; k < 4; ++k) {
    joints.position[3 * k + 1] = .8;
    joints.position[3 * k + 2] = -1.6;
  }
  ImuSample imu;
  imu.valid = true;
  imu.linear_acceleration[2] = 9.81;
  ContactSupport a, b;
  a.contact.fill(true);
  a.height_valid.fill(true);
  b = a;
  b.foot_center_height.fill(.2);
  EstimatedState x, y;
  for (int k = 0; k < 100; ++k) {
    imu.stamp_seconds = k * .004;
    x = flat.Update(joints, imu, a, .004);
    y = raised.Update(joints, imu, b, .004);
  }
  ASSERT_TRUE(x.valid);
  ASSERT_TRUE(y.valid);
  EXPECT_NEAR(y.position.z() - x.position.z(), .2, 1e-4);
  b.height_valid.fill(false);
  auto before = y.position.z();
  for (int k = 0; k < 100; ++k)
    y = raised.Update(joints, imu, b, .004);
  EXPECT_NEAR(y.position.z(), before, .002);
}

TEST(Executor, BlockedSwingWithoutVerticalContactCannotReachConfirm) {
  FootstepExecutor ex;
  auto r = Stance();
  ex.Reset(r);
  PrecisionStep s;
  s.target = r.foot[0] + Eigen::Vector3d(.1, 0, 0);
  s.body = r.body;
  s.shift = .2;
  s.swing = 1.;
  ex.Start(s, 0);
  std::array<ContactEstimate, 4> c;
  for (auto &v : c)
    v.valid = v.loaded = true;
  c[0].loaded = false;
  for (double t = 0; t < 1.; t += .001)
    ex.Update(t, c, r.foot);
  EXPECT_EQ(ex.error_code(), PrecisionError::SWING_TRACKING_ERROR);
  EXPECT_FALSE(ex.reference().contact[0]);
}

TEST(Executor, IdleSupportLossIsNotIgnored) {
  FootstepExecutor ex;
  ex.Reset(Stance());
  std::array<ContactEstimate, 4> c;
  for (auto &v : c)
    v.valid = v.loaded = true;
  c[2].loaded = false;
  ex.Update(1., c, Stance().foot);
  EXPECT_EQ(ex.error_code(), PrecisionError::SUPPORT_UNCONFIRMED);
}

TEST(Executor,
     CompletedStepRetainsValidatedTerminalBodyWithoutSkippingContact) {
  FootstepExecutor ex;
  auto r = Stance();
  ex.Reset(r);
  PrecisionStep s;
  s.target = r.foot[0] + Eigen::Vector3d(.025, 0, 0);
  s.body = r.body + Eigen::Vector3d(-.03, .04, 0);
  s.has_body_finish = true;
  s.body_finish = r.body + Eigen::Vector3d(.00625, 0, 0);
  s.shift = .2;
  s.swing = .4;
  ex.Start(s, 0);
  std::array<ContactEstimate, 4> contact;
  for (auto &v : contact)
    v.valid = v.loaded = true;
  bool saw_confirm = false;
  for (double t = 0; t < 2.; t += .001) {
    contact[0].loaded = ex.phase() != StepPhase::SWING;
    contact[0].estimated_force.z() = contact[0].loaded ? 30. : 0.;
    const auto feet = ex.reference().foot;
    ex.Update(t, contact, feet);
    saw_confirm |= ex.phase() == StepPhase::CONFIRM;
  }
  ASSERT_TRUE(saw_confirm);
  ASSERT_EQ(ex.phase(), StepPhase::DONE) << ex.error();
  EXPECT_NEAR((ex.reference().body - s.body_finish).norm(), 0., 1e-12);
  EXPECT_NEAR(ex.reference().body_velocity.norm(), 0., 1e-12);
  EXPECT_NEAR(ex.reference().body_acceleration.norm(), 0., 1e-12);
}

static PrecisionStep CoordinatedStep(const WholeBodyReference &r) {
  PrecisionStep s;
  s.id = 72;
  s.target = r.foot[0] + Eigen::Vector3d(.02, 0, 0);
  s.body = r.body + Eigen::Vector3d(-.01, .01, 0);
  s.has_body_finish = true;
  s.body_finish = s.body;
  s.shift = 2;
  s.swing = 4;
  s.trajectory.size = 4;
  s.trajectory.lift_index = 1;
  s.trajectory.knots[0] = {0., r.body, r.euler, r.foot[0]};
  s.trajectory.knots[1] = {2., s.body, r.euler, r.foot[0]};
  s.trajectory.knots[2] = {4., s.body, r.euler + Eigen::Vector3d(.02, 0, 0),
                           r.foot[0] + Eigen::Vector3d(.01, 0, .04)};
  s.trajectory.knots[3] = {6., s.body, r.euler + Eigen::Vector3d(.02, 0, 0),
                           s.target};
  return s;
}
TEST(CoordinatedReference, BoundsAndKnotDerivatives) {
  auto r = Stance();
  auto s = CoordinatedStep(r);
  ASSERT_TRUE(ValidTrajectory(s, r, {}));
  for (double t : {0., 2., 4.}) {
    SampleTrajectory(s.trajectory, t, true, 0, r);
    EXPECT_LT(r.body_velocity.norm() + r.euler_velocity.norm() +
                  r.velocity[0].norm(),
              1e-12);
    EXPECT_LT(r.body_acceleration.norm() + r.euler_acceleration.norm() +
                  r.acceleration[0].norm(),
              1e-12);
  }
  EXPECT_LT((r.foot[0] - s.target).norm(), 1e-12);
  s.trajectory.size = 193;
  EXPECT_FALSE(ValidTrajectory(s, Stance(), {}));
  s.trajectory.size = 4;
  auto invalid_duration = s;
  invalid_duration.shift = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ValidTrajectory(invalid_duration, Stance(), {}));
  invalid_duration = s;
  invalid_duration.swing += .1;
  EXPECT_FALSE(ValidTrajectory(invalid_duration, Stance(), {}));
  s.trajectory.knots[2].time = 2.;
  EXPECT_FALSE(ValidTrajectory(s, Stance(), {}));
  s = CoordinatedStep(Stance());
  s.trajectory.knots[2].euler.y() = .36;
  EXPECT_FALSE(ValidTrajectory(s, Stance(), {}));
  s = CoordinatedStep(Stance());
  s.trajectory.knots[1].foot.z() += .02;
  EXPECT_FALSE(ValidTrajectory(s, Stance(), {}));
}
TEST(CoordinatedReference,
     TimingNeverFabricatesContactAndAbortStopsAngularReference) {
  auto r = Stance();
  auto s = CoordinatedStep(r);
  FootstepExecutor ex;
  ex.Reset(r);
  ex.Start(s, 0);
  std::array<ContactEstimate, 4> c;
  for (auto &v : c)
    v.valid = v.loaded = true;
  for (int k = 0; k < 750; ++k) {
    const double t = k * .01;
    c[0].loaded = t < 2.;
    ex.Update(t, c, ex.reference().foot);
  }
  EXPECT_EQ(ex.phase(), StepPhase::HOLD);
  EXPECT_EQ(ex.error_code(), PrecisionError::TOUCHDOWN_TIMEOUT);
  EXPECT_EQ(ex.id(), 72u);
  EXPECT_TRUE(ex.reference().euler_velocity.isZero());
  EXPECT_TRUE(ex.reference().euler_acceleration.isZero());
}
TEST(CoordinatedReference, ConfirmedTouchdownKeepsTerminalBodyAndOrientation) {
  auto r = Stance();
  auto s = CoordinatedStep(r);
  FootstepExecutor ex;
  ex.Reset(r);
  ex.Start(s, 0);
  std::array<ContactEstimate, 4> c;
  for (auto &v : c)
    v.valid = v.loaded = true;
  for (int k = 0; k < 900; ++k) {
    const double t = k * .01;
    c[0].loaded = t < 2. || t > 6.02;
    c[0].estimated_force.z() = c[0].loaded ? 30. : 0.;
    ex.Update(t, c, ex.reference().foot);
  }
  EXPECT_EQ(ex.phase(), StepPhase::DONE);
  EXPECT_LT((ex.reference().body - s.body_finish).norm(), 1e-12);
  EXPECT_LT((ex.reference().euler - s.trajectory.knots[3].euler).norm(), 1e-12);
}
