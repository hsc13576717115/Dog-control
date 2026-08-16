//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include "legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

// Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
class WbcBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period);

  bool lastSolverSucceeded() const { return lastSolverSucceeded_; }
  scalar_t lastEqualityResidual() const { return lastEqualityResidual_; }
  scalar_t lastInequalityViolation() const { return lastInequalityViolation_; }

 protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }
  size_t getNumPhysicalDecisionVars() const { return numPhysicalDecisionVars_; }

  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateJointLimitsTask();
  Task formulateJointLimitSlackTask();
  Task formulateNoContactMotionTask();
  Task formulateFrictionConeTask();
  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  Task formulateSwingLegTask();
  Task formulateContactForceTask(const vector_t& inputDesired) const;

  size_t numDecisionVars_;
  size_t numPhysicalDecisionVars_;
  size_t numJointLimitSlacks_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  vector_t qMeasured_, vMeasured_, inputLast_;
  matrix_t j_, dj_;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
  scalar_t jointPositionMargin_{0.10}, jointLimitHorizon_{0.05};
  bool lastSolverSucceeded_ = false;
  scalar_t lastEqualityResidual_ = 0.0;
  scalar_t lastInequalityViolation_ = 0.0;
};

}  // namespace legged
