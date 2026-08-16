//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/WbcBase.h"

#include <algorithm>
#include <cmath>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

namespace legged {
WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      inputLast_(vector_t::Zero(info_.inputDim)),
      eeKinematics_(eeKinematics.clone()) {
  numPhysicalDecisionVars_ = info_.generalizedCoordinatesNum +
                             3 * info_.numThreeDofContacts +
                             info_.actuatedDofNum;
  numJointLimitSlacks_ = 2 * info_.actuatedDofNum;
  numDecisionVars_ = numPhysicalDecisionVars_ + numJointLimitSlacks_;
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
}

vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                         scalar_t /*period*/) {
  contactFlag_ = modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
  }

  updateMeasured(rbdStateMeasured);
  updateDesired(stateDesired, inputDesired);

  return {};
}

void WbcBase::updateMeasured(const vector_t& rbdStateMeasured) {
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // For floating base EoM task
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeasured_);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // For not contact motion task
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }
}

void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired) {
  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();

  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  pinocchio::forwardKinematics(model, data, qDesired);
  pinocchio::computeJointJacobians(model, data, qDesired);
  pinocchio::updateFramePlacements(model, data);
  updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
  pinocchio::forwardKinematics(model, data, qDesired, vDesired);
}

Task WbcBase::formulateFloatingBaseEomTask() {
  auto& data = pinocchioInterfaceMeasured_.getData();

  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

  matrix_t a(info_.generalizedCoordinatesNum, numDecisionVars_);
  a.setZero();
  a.block(0, 0, info_.generalizedCoordinatesNum,
          info_.generalizedCoordinatesNum) = data.M;
  a.block(0, info_.generalizedCoordinatesNum,
          info_.generalizedCoordinatesNum,
          3 * info_.numThreeDofContacts) = -j_.transpose();
  a.block(0,
          info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts,
          info_.generalizedCoordinatesNum, info_.actuatedDofNum) =
      -s.transpose();
  vector_t b = -data.nle;

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateTorqueLimitsTask() {
  matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = i;
  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = -i;
  vector_t f(2 * info_.actuatedDofNum);
  for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l) {
    f.segment<3>(3 * l) = torqueLimits_;
  }

  return {matrix_t(), vector_t(), d, f};
}

Task WbcBase::formulateJointLimitsTask() {
  const size_t jointCount = info_.actuatedDofNum;
  matrix_t d(4 * jointCount, numDecisionVars_);
  d.setZero();
  vector_t f(4 * jointCount);
  f.setZero();
  const auto& model = pinocchioInterfaceMeasured_.getModel();
  const scalar_t horizon = std::max<scalar_t>(jointLimitHorizon_, 1e-3);
  const scalar_t accelerationScale = 2.0 / (horizon * horizon);

  for (size_t i = 0; i < jointCount; ++i) {
    const Eigen::Index qIndex = static_cast<Eigen::Index>(6 + i);
    const scalar_t lower = model.lowerPositionLimit(qIndex) + jointPositionMargin_;
    const scalar_t upper = model.upperPositionLimit(qIndex) - jointPositionMargin_;
    const scalar_t predictedWithoutAccel =
        qMeasured_(qIndex) + horizon * vMeasured_(qIndex);

    // q(T) = q + qdot*T + 0.5*qdd*T^2 must remain inside the
    // position limits. The QP is resolved every WBC cycle, so this acts as a
    // velocity-aware braking constraint before the hard URDF boundary.
    const Eigen::Index upperSlack = static_cast<Eigen::Index>(
        numPhysicalDecisionVars_ + 2 * i);
    const Eigen::Index lowerSlack = upperSlack + 1;
    d(4 * i, qIndex) = 1.0;
    d(4 * i, upperSlack) = -1.0;
    f(4 * i) = accelerationScale * (upper - predictedWithoutAccel);
    d(4 * i + 1, qIndex) = -1.0;
    d(4 * i + 1, lowerSlack) = -1.0;
    f(4 * i + 1) = accelerationScale * (predictedWithoutAccel - lower);
    d(4 * i + 2, upperSlack) = -1.0;
    d(4 * i + 3, lowerSlack) = -1.0;
  }
  return {matrix_t(), vector_t(), d, f};
}

Task WbcBase::formulateJointLimitSlackTask() {
  matrix_t a(numJointLimitSlacks_, numDecisionVars_);
  a.setZero();
  a.rightCols(numJointLimitSlacks_).setIdentity();
  return {a, vector_t::Zero(numJointLimitSlacks_), matrix_t(), vector_t()};
}

Task WbcBase::formulateNoContactMotionTask() {
  matrix_t a(3 * numContacts_, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    if (contactFlag_[i]) {
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateFrictionConeTask() {
  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
  }
  vector_t b(a.rows());
  b.setZero();

  matrix_t frictionPyramic(5, 3);  // clang-format off
  frictionPyramic << 0, 0, -1,
                     1, 0, -frictionCoeff_,
                    -1, 0, -frictionCoeff_,
                     0, 1, -frictionCoeff_,
                     0,-1, -frictionCoeff_;  // clang-format on

  matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (contactFlag_[i]) {
      d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
    }
  }
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return {a, b, d, f};
}

Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
  inputLast_ = inputDesired;
  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

  const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
  const Matrix6 Ab = A.template leftCols<6>();
  const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info_.actuatedDofNum);
  const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);
  Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
  centroidalMomentumRate.noalias() -= ADot * vDesired;
  centroidalMomentumRate.noalias() -= Aj * jointAccel;

  Vector6 b = AbInv * centroidalMomentumRate;

  const vector3_t positionError =
      qDesired.head<3>() - qMeasured_.head<3>();
  vector3_t orientationError =
      qDesired.segment<3>(3) - qMeasured_.segment<3>(3);
  orientationError(0) = std::remainder(
      orientationError(0), 2.0 * std::acos(-1.0));
  b.head<3>() +=
      basePositionKp_.cwiseProduct(positionError) +
      basePositionKd_.cwiseProduct(
          vDesired.head<3>() - vMeasured_.head<3>());
  b.tail<3>() +=
      baseOrientationKp_.cwiseProduct(orientationError) +
      baseOrientationKd_.cwiseProduct(
          vDesired.segment<3>(3) - vMeasured_.segment<3>(3));

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateSwingLegTask() {
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
  std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const {
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  b = inputDesired.head(a.rows());

  return {a, b, matrix_t(), vector_t()};
}

void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // Load task file
  torqueLimits_ = vector_t(info_.actuatedDofNum / 4);
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
  if (verbose) {
    std::cerr << "\n #### Torque Limits Task:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### HAA HFE KFE: " << torqueLimits_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose) {
    std::cerr << "\n #### Friction Cone Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }
  prefix = "swingLegTask.";
  if (verbose) {
    std::cerr << "\n #### Swing Leg Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);
  loadData::loadEigenMatrix(
      taskFile, "baseAccelTask.positionKp", basePositionKp_);
  loadData::loadEigenMatrix(
      taskFile, "baseAccelTask.positionKd", basePositionKd_);
  loadData::loadEigenMatrix(
      taskFile, "baseAccelTask.orientationKp", baseOrientationKp_);
  loadData::loadEigenMatrix(
      taskFile, "baseAccelTask.orientationKd", baseOrientationKd_);
  prefix = "jointLimitsTask.";
  loadData::loadPtreeValue(
      pt, jointPositionMargin_, prefix + "positionMargin", verbose);
  loadData::loadPtreeValue(
      pt, jointLimitHorizon_, prefix + "predictionHorizon", verbose);
}

}  // namespace legged
