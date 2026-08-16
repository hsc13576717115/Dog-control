//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WeightedWbc.h"

#include <limits>
#include <qpOASES.hpp>

namespace legged {

vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t period) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // Constraints
  Task constraints = formulateConstraints();
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints);  // clang-format off
  A << constraints.a_,
       constraints.d_;

  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  ubA << constraints.b_,
         constraints.f_;  // clang-format on

  // Cost
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

  // Solve
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  // A cold-started stance QP can need more than 20 active-set changes during
  // the position-to-WBC handoff. The controller still rejects failed solves
  // and checks the resulting equality and inequality residuals.
  int nWsr = 100;

  const auto status = qpProblem.init(
      H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
  vector_t qpSol(getNumDecisionVars());
  qpSol.setZero();
  lastSolverSucceeded_ =
      status == qpOASES::SUCCESSFUL_RETURN &&
      qpProblem.getPrimalSolution(qpSol.data()) == qpOASES::SUCCESSFUL_RETURN &&
      qpSol.allFinite();
  if (!lastSolverSucceeded_) {
    lastEqualityResidual_ = std::numeric_limits<scalar_t>::infinity();
    lastInequalityViolation_ = std::numeric_limits<scalar_t>::infinity();
    return qpSol;
  }
  lastEqualityResidual_ =
      constraints.a_.rows() > 0
          ? (constraints.a_ * qpSol - constraints.b_).lpNorm<Eigen::Infinity>()
          : 0.0;
  lastInequalityViolation_ =
      constraints.d_.rows() > 0
          ? (constraints.d_ * qpSol - constraints.f_).cwiseMax(0.0).maxCoeff()
          : 0.0;
  return qpSol.head(getNumPhysicalDecisionVars());
}

Task WeightedWbc::formulateConstraints() {
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() +
         formulateJointLimitsTask() + formulateFrictionConeTask() +
         formulateNoContactMotionTask();
}

Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
         formulateContactForceTask(inputDesired) * weightContactForce_ +
         formulateJointLimitSlackTask() * weightJointLimitSlack_;
}

void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
  loadData::loadPtreeValue(
      pt, weightJointLimitSlack_, prefix + "jointLimitSlack", verbose);
}

}  // namespace legged
