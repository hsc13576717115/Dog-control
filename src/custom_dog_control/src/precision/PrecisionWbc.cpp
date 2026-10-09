#include "custom_dog_control/precision/PrecisionWbc.hpp"
#include <pinocchio/algorithm/frames.hpp>
namespace custom_dog_control {
legged::Task PrecisionWbc::formulateConstraints() {
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
  b.segment<3>(3) = config_.orientation_kp * angle_error -
                    config_.orientation_kd * vMeasured_.segment<3>(3);
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
