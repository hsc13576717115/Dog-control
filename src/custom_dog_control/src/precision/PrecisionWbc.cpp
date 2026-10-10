#include "custom_dog_control/precision/PrecisionWbc.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
namespace custom_dog_control {
legged::Task PrecisionWbc::formulateConstraints() {
  if (config_.sole_rolling_model > .5) {
    const auto &m = pinocchioInterfaceMeasured_.getModel();
    auto &d = pinocchioInterfaceMeasured_.getData();
    Eigen::Matrix3d cross_normal;
    cross_normal << 0, -1, 0, 1, 0, 0, 0, 0, 0;
    for (size_t f = 0; f < 4; ++f)
      if (contactFlag_[f]) {
        Eigen::Matrix<double, 6, Eigen::Dynamic> jac(6, m.nv),
            derivative(6, m.nv);
        jac.setZero();
        derivative.setZero();
        pinocchio::getFrameJacobian(m, d, info_.endEffectorFrameIndices[f],
                                    pinocchio::LOCAL_WORLD_ALIGNED, jac);
        pinocchio::getFrameJacobianTimeVariation(
            m, d, info_.endEffectorFrameIndices[f],
            pinocchio::LOCAL_WORLD_ALIGNED, derivative);
        // v_contact = v_center + omega x (-r*n). The same Jacobian maps
        // contact forces into generalized forces in the EoM/torque constraints.
        j_.middleRows(3 * f, 3) +=
            sole_radius_ * cross_normal * jac.bottomRows(3);
        dj_.middleRows(3 * f, 3) +=
            sole_radius_ * cross_normal * derivative.bottomRows(3);
      }
  }
  // Position feedback stabilizes the actual support anchors, not a flat
  // posture.
  ocs2::matrix_t a = ocs2::matrix_t::Zero(3 * numContacts_, numDecisionVars_);
  ocs2::vector_t b = ocs2::vector_t::Zero(3 * numContacts_);
  size_t row = 0;
  for (size_t f = 0; f < 4; ++f)
    if (contactFlag_[f]) {
      const auto &d = pinocchioInterfaceMeasured_.getData();
      auto p = d.oMf[info_.endEffectorFrameIndices[f]].translation();
      a.block(3 * row, 0, 3, info_.generalizedCoordinatesNum) =
          j_.middleRows(3 * f, 3);
      b.segment<3>(3 * row) =
          config_.stance_kp * (ref_.foot[f] - p) -
          config_.stance_kd * j_.middleRows(3 * f, 3) * vMeasured_ -
          dj_.middleRows(3 * f, 3) * vMeasured_;
      ++row;
    }
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() +
         formulateJointLimitsTask() + formulateFrictionConeTask() +
         legged::Task(a, b, {}, {});
}
legged::Task PrecisionWbc::formulateWeightedTasks(const ocs2::vector_t &,
                                                  const ocs2::vector_t &input,
                                                  double) {
  ocs2::matrix_t a = ocs2::matrix_t::Zero(18, numDecisionVars_);
  ocs2::vector_t b = ocs2::vector_t::Zero(18);
  a.block<6, 6>(0, 0).setIdentity();
  b.head<3>() = ref_.body_acceleration +
                config_.body_kp * (ref_.body - qMeasured_.head<3>()) +
                config_.body_kd * (ref_.body_velocity - vMeasured_.head<3>());
  Eigen::Vector3d angle_error = ref_.euler - qMeasured_.segment<3>(3);
  for (int k = 0; k < 3; ++k)
    angle_error(k) =
        std::atan2(std::sin(angle_error(k)), std::cos(angle_error(k)));
  b.segment<3>(3) =
      ref_.euler_acceleration + config_.orientation_kp * angle_error +
      config_.orientation_kd * (ref_.euler_velocity - vMeasured_.segment<3>(3));
  for (size_t f = 0; f < 4; ++f)
    if (!contactFlag_[f]) {
      const auto &d = pinocchioInterfaceMeasured_.getData();
      auto p = d.oMf[info_.endEffectorFrameIndices[f]].translation();
      a.block(6 + 3 * f, 0, 3, info_.generalizedCoordinatesNum) =
          config_.swing_weight * j_.middleRows(3 * f, 3);
      b.segment<3>(6 + 3 * f) =
          config_.swing_weight *
          (ref_.acceleration[f] + config_.swing_kp * (ref_.foot[f] - p) +
           config_.swing_kd *
               (ref_.velocity[f] - j_.middleRows(3 * f, 3) * vMeasured_) -
           dj_.middleRows(3 * f, 3) * vMeasured_);
    }
  return legged::Task(a, b, {}, {}) +
         formulateContactForceTask(input) * config_.force_weight +
         formulateJointLimitSlackTask() * config_.joint_slack_weight;
}
} // namespace custom_dog_control
