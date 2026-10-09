#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"
#include "custom_dog_control/precision/ContactObserver.hpp"
#include "custom_dog_control/precision/FootstepExecutor.hpp"
#include <gtest/gtest.h>
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
    ex.Update(t, c, r.foot);
  EXPECT_EQ(ex.phase(), StepPhase::HOLD);
  EXPECT_EQ(ex.error(), "touchdown_timeout");
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
  EXPECT_EQ(ex.error(), "early_contact");
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
