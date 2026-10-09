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
      b.segment<3>(3 * row) = 120. * (ref_.foot[f] - p) -
                              24. * j_.middleRows(3 * f, 3) * vMeasured_ -
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
                100. * (ref_.body - qMeasured_.head<3>()) +
                25. * (ref_.body_velocity - vMeasured_.head<3>());
  Eigen::Vector3d angle_error = ref_.euler - qMeasured_.segment<3>(3);
  for (int k = 0; k < 3; ++k)
    angle_error(k) =
        std::atan2(std::sin(angle_error(k)), std::cos(angle_error(k)));
  b.segment<3>(3) = 120. * angle_error - 25. * vMeasured_.segment<3>(3);
  for (size_t f = 0; f < 4; ++f)
    if (!contactFlag_[f]) {
      const auto &d = pinocchioInterfaceMeasured_.getData();
      auto p = d.oMf[info_.endEffectorFrameIndices[f]].translation();
      a.block(6 + 3 * f, 0, 3, info_.generalizedCoordinatesNum) =
          5. * j_.middleRows(3 * f, 3);
      b.segment<3>(6 + 3 * f) =
          5. *
          (ref_.acceleration[f] + 250. * (ref_.foot[f] - p) +
           30. * (ref_.velocity[f] - j_.middleRows(3 * f, 3) * vMeasured_) -
           dj_.middleRows(3 * f, 3) * vMeasured_);
    }
  return legged::Task(a, b, {}, {}) + formulateContactForceTask(input) * .005 +
         formulateJointLimitSlackTask() * 100.;
}
} // namespace custom_dog_control
