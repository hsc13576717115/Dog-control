#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/model/StaticSupport.hpp"
#include "custom_dog_control/precision/PrecisionModel.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <pinocchio/multibody/model.hpp>
using namespace custom_dog_control;
TEST(StaticSupport, StandingCanBalanceAndNoContactsCannot) {
  RobotModel robot(CUSTOM_DOG_CONTROL_CANONICAL_URDF, {});
  const auto &m = robot.pin.getModel();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m.nq);
  q[2] = .3;
  for (size_t k = 0; k < 12; ++k) {
    const auto idx =
        m.joints[m.getJointId(std::string(kJointNames[k]))].idx_q();
    q[idx] = k % 3 == 0 ? 0 : (k % 3 == 1 ? .8 : -1.6);
  }
  const auto standing =
      CheckStaticSupport(robot, q, {true, true, true, true}, .5);
  ASSERT_TRUE(standing.feasible);
  EXPECT_NEAR(standing.forces(2) + standing.forces(5) + standing.forces(8) +
                  standing.forces(11),
              robot.info.robotMass * 9.81, 1e-5);
  EXPECT_FALSE(
      CheckStaticSupport(robot, q, {false, false, false, false}, .5).feasible);
  EXPECT_FALSE(
      CheckStaticSupport(robot, q, {true, false, false, false}, .5).feasible);
  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      CheckStaticSupport(robot, q, {true, true, true, true}, .5).feasible);
}

TEST(StaticSupport, ShiftedTripodHasAStaticForceSolution) {
  RobotModel robot(CUSTOM_DOG_CONTROL_CANONICAL_URDF, {});
  PrecisionModel geometry(robot.pin, robot.info,
                          CUSTOM_DOG_CONTROL_CANONICAL_URDF);
  JointSample seed;
  seed.valid.fill(1.);
  for (size_t f = 0; f < 4; ++f) {
    seed.position[3 * f + 1] = .8;
    seed.position[3 * f + 2] = -1.6;
  }
  EstimatedState state;
  state.position.z() = .3;
  geometry.Measure(seed, state, 0);
  WholeBodyReference ref;
  ref.foot = geometry.feet();
  ref.body = {-.04, .05, .3};
  Eigen::VectorXd q, dq;
  ASSERT_TRUE(geometry.Inverse(ref, seed, q, dq));
  const auto result =
      CheckStaticSupport(robot, q, {false, true, true, true}, .5);
  EXPECT_TRUE(result.feasible)
      << result.solver_return_code << " residual=" << result.residual;
}

TEST(CollisionGeometry, CadSelfCheckDoesNotUseOverlappingStandingPrimitives) {
  RobotModel robot(CUSTOM_DOG_CONTROL_CANONICAL_URDF, {});
  PrecisionModel geometry(robot.pin, robot.info,
                          CUSTOM_DOG_CONTROL_CANONICAL_URDF);
  const auto &m = robot.pin.getModel();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(m.nq);
  q[2] = .29;
  const std::array<double, 12> nominal{
      -.14, .873118638, -1.776967680, .14, .872802956, -1.777230490,
      -.14, .872609587, -1.777385860, .14, .870485702, -1.779097810};
  for (size_t k = 0; k < 12; ++k)
    q[6 + geometry.slot(k)] = nominal[k];
  CollisionWitness witness;
  EXPECT_TRUE(geometry.SelfCollisionFree(q, &witness))
      << witness.first << " / " << witness.second;
  // A solid box enclosing the entire robot must still be rejected. CAD self
  // checks never remove or replace environmental body/leg collision shapes.
  EXPECT_FALSE(geometry.CollisionFree(q, {0, 0, 0}, 1., {2., 2.}, &witness));
  EXPECT_FALSE(witness.first.empty());
}

TEST(InverseKinematics,
     AcceptedConfigurationMustMeetToleranceAfterJointClamping) {
  RobotModelConfig mc;
  PrecisionConfig config;
  RobotModel robot(CUSTOM_DOG_CONTROL_CANONICAL_URDF, mc);
  PrecisionModel geometry(robot.pin, robot.info,
                          CUSTOM_DOG_CONTROL_CANONICAL_URDF, mc, config);
  WholeBodyReference ref;
  ref.body = {-.17, .065, .4};
  ref.euler = {0., -.16, .08};
  ref.foot = {
      {{.3, -.2, .266}, {-.05, .2, .226}, {-.29, -.2, .226}, {-.29, .2, .226}}};
  // A retained near-limit stone path used to return success and then move
  // its calf against the joint margin, leaving 0.862 mm FK error (>0.5 mm).
  const std::array<double, 12> prior_pin_joints{
      -.09881260711648092, 1.4634297349395584, -2.065136319051574,
      -.6104690648527233,  -.3015127387646857, -.9372881940864646,
      -.10108709057551842, .9009112716241452,  -2.362725721846962,
      -.7030817386112004,  .7690480012261263,  -1.890771988539135};
  JointSample seed;
  seed.valid.fill(1.);
  for (size_t k = 0; k < 12; ++k)
    seed.position[k] = prior_pin_joints[geometry.slot(k)];
  Eigen::VectorXd q, dq;
  if (geometry.Inverse(ref, seed, q, dq)) {
    EstimatedState state;
    state.position = ref.body;
    state.euler_zyx = ref.euler;
    for (size_t k = 0; k < 12; ++k)
      seed.position[k] = q[6 + geometry.slot(k)];
    geometry.Measure(seed, state, 0.);
    for (size_t f = 0; f < 4; ++f)
      EXPECT_LT((geometry.feet()[f] - ref.foot[f]).norm(),
                config.ik_tolerance_m);
  }
}

TEST(StaticSupport, WallFrontLandingPoseIsNotAWholeBodyTraversalCertificate) {
  RobotModel robot(CUSTOM_DOG_CONTROL_CANONICAL_URDF, {});
  PrecisionModel geometry(robot.pin, robot.info,
                          CUSTOM_DOG_CONTROL_CANONICAL_URDF);
  Eigen::VectorXd q(18);
  // Feasible endpoint found by offline constrained pose search. The path from
  // the approach stance and rear-leg traversal are deliberately NOT asserted.
  q << -.142795403648193, .112900871583629, .354997318734503, .618830985349756,
      -.00268995127315339, .00129143165478366, -.287717823111853,
      .948837562832283, -1.30607102488952, -.116146253844465,
      -.0118357814977808, -.88775804096, .337583520365917, .519519128149904,
      -.88775804096, -.269528071989158, .587877279106044, -1.03890072083602;
  JointSample joints;
  joints.valid.fill(1.);
  for (size_t k = 0; k < 12; ++k)
    joints.position[k] = q[6 + geometry.slot(k)];
  EstimatedState state;
  state.position = q.head<3>();
  state.euler_zyx = q.segment<3>(3);
  geometry.Measure(joints, state, 0.);
  const std::array<Eigen::Vector3d, 4> expected{
      {{.24667623585, .17124508746, .026},
       {-.1, .2, .026},
       {-.18, -.2, .026},
       {-.47, .2, .026}}};
  for (size_t f = 0; f < 4; ++f)
    EXPECT_LT((geometry.feet()[f] - expected[f]).norm(), 1e-6);
  CollisionWitness collision;
  EXPECT_TRUE(geometry.SelfCollisionFree(q, &collision))
      << collision.first << "/" << collision.second;
  EXPECT_TRUE(geometry.CollisionFree(q, {0, 0, 0}, .3, {.08, .8}, &collision))
      << collision.first;
  EXPECT_TRUE(geometry.CollisionFree(q, {0, 0, 0}, 0., {10., 10.}, &collision))
      << collision.first;
  const auto result =
      CheckStaticSupport(robot, q, {false, true, true, true}, .5, .026);
  EXPECT_TRUE(result.feasible) << result.solver_return_code;
  const auto &model = robot.pin.getModel();
  for (int k = 6; k < 18; ++k) {
    EXPECT_GE(q[k], model.lowerPositionLimit[k] + .05 - 1e-12);
    EXPECT_LE(q[k], model.upperPositionLimit[k] - .05 + 1e-12);
  }
}
