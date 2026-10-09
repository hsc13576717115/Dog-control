#include "custom_dog_control/precision/PrecisionExecutionCore.hpp"
#include "custom_dog_control/precision/PrecisionModel.hpp"
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
using namespace custom_dog_control;
namespace {
double fake_time = 0, increment = 0;
double Clock() {
  fake_time += increment;
  return fake_time;
}
struct Fixture {
  RobotModelConfig m;
  PrecisionConfig c;
  RobotModel robot{CUSTOM_DOG_CONTROL_CANONICAL_URDF, m};
  PrecisionExecutionCore core{robot,
                              CUSTOM_DOG_CONTROL_CANONICAL_URDF,
                              std::string(CUSTOM_DOG_CONTROL_SOURCE_DIR) +
                                  "/config/nmpc/task.info",
                              m,
                              c,
                              Clock};
  JointSample joint;
  ImuSample imu;
  double now = 0;
  Fixture() {
    increment = 0;
    fake_time = 10.;
    joint.valid.fill(1.);
    joint.stamp_valid = true;
    imu.valid = true;
    imu.linear_acceleration[2] = 9.81;
    auto pin = robot.pin;
    const auto &model = pin.getModel();
    auto &data = pin.getData();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq),
                    zero = Eigen::VectorXd::Zero(model.nv);
    q[2] = m.initial_height_m;
    std::array<int, 12> slots;
    for (size_t k = 0; k < 12; ++k) {
      slots[k] =
          model.joints[model.getJointId(std::string(kJointNames[k]))].idx_q();
      joint.position[k] = k % 3 == 0 ? 0. : (k % 3 == 1 ? .8 : -1.6);
      q[slots[k]] = joint.position[k];
    }
    const Eigen::VectorXd gravity = pinocchio::rnea(model, data, q, zero, zero);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    for (size_t f = 0; f < 4; ++f) {
      Eigen::Matrix<double, 6, Eigen::Dynamic> jac(6, model.nv);
      jac.setZero();
      pinocchio::getFrameJacobian(
          model, data, model.getFrameId(std::string(kFootFrameNames[f])),
          pinocchio::LOCAL_WORLD_ALIGNED, jac);
      for (size_t j = 0; j < 3; ++j) {
        const auto k = f * 3 + j;
        joint.effort[k] = gravity[slots[k]] -
                          jac(2, slots[k]) * robot.info.robotMass * 9.81 / 4;
      }
    }
  }
  bool Tick(bool refresh_joint = true, bool refresh_imu = true) {
    now += .001;
    if (refresh_joint)
      joint.stamp_seconds = now;
    if (refresh_imu)
      imu.stamp_seconds = now;
    HybridJointCommand out;
    return core.Update(now, .001, joint, imu, false, {}, false, out);
  }
  bool Settle() {
    for (int k = 0; k < 1500; ++k) {
      if (!Tick())
        return false;
      if (core.snapshot().ready)
        return true;
    }
    return false;
  }
};
} // namespace
TEST(ExecutionFaults, FrozenJointAcquisitionTripsEvenWhileUpdateKeepsRunning) {
  Fixture f;
  ASSERT_TRUE(f.Settle()) << ErrorName(f.core.snapshot().error);
  for (int k = 0; k < 110; ++k)
    f.Tick(false, true);
  EXPECT_EQ(f.core.snapshot().error, PrecisionError::INVALID_STATE);
  EXPECT_FALSE(f.core.snapshot().estimate.valid);
  EXPECT_FALSE(f.core.snapshot().ready);
}
TEST(ExecutionFaults, OutOfOrderImuTripsInsideFreshnessWindow) {
  Fixture f;
  ASSERT_TRUE(f.Settle());
  f.imu.stamp_seconds -= .002;
  EXPECT_FALSE(f.Tick(true, false));
  EXPECT_EQ(f.core.snapshot().error, PrecisionError::INVALID_STATE);
}
TEST(ExecutionFaults, SolverDeadlineFaultIsLatched) {
  Fixture f;
  ASSERT_TRUE(f.Settle());
  // Inject only the elapsed-clock boundary. This tests fault routing after a
  // real WBC solve, NOT host WCET, scheduling, or a physically delayed motor.
  increment = .021;
  EXPECT_FALSE(f.Tick());
  EXPECT_EQ(f.core.snapshot().error, PrecisionError::WBC_INVALID_OR_TIMEOUT);
  EXPECT_GE(f.core.snapshot().solve_ms, 20.);
  increment = 0;
  EXPECT_FALSE(f.Tick());
  EXPECT_FALSE(f.core.snapshot().ready);
}

TEST(ExecutionFaults, OutOfOrderJointTripsInsideFreshnessWindow) {
  Fixture f;
  ASSERT_TRUE(f.Settle());
  f.joint.stamp_seconds -= .002;
  EXPECT_FALSE(f.Tick(false, true));
  EXPECT_EQ(f.core.snapshot().error, PrecisionError::INVALID_STATE);
}
