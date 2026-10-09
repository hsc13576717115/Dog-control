#include "custom_dog_control/model/StaticSupport.hpp"
#include "custom_dog_control/control/ControlTypes.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <qpOASES.hpp>
namespace custom_dog_control {
StaticSupportResult CheckStaticSupport(const RobotModel &robot,
                                       const Eigen::VectorXd &q,
                                       const std::array<bool, 4> &contacts,
                                       double friction, double sole_radius) {
  StaticSupportResult out;
  const auto &m = robot.pin.getModel();
  if (q.size() != m.nq || m.nv != 18 || !q.allFinite() ||
      !std::isfinite(friction) || friction <= 0 ||
      !std::isfinite(sole_radius) || sole_radius < 0)
    return out;
  auto pin = robot.pin;
  auto &d = pin.getData();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(m.nv);
  const Eigen::VectorXd gravity = pinocchio::rnea(m, d, q, zero, zero);
  pinocchio::computeJointJacobians(m, d, q);
  pinocchio::updateFramePlacements(m, d);
  std::vector<size_t> active;
  for (size_t f = 0; f < 4; ++f)
    if (contacts[f])
      active.push_back(f);
  if (active.empty()) {
    out.infeasible = true;
    return out;
  }
  const int variables = 3 * active.size(), constraints = 18 + 4 * active.size();
  using Matrix =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  Matrix H = Matrix::Identity(variables, variables),
         A = Matrix::Zero(constraints, variables);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(variables),
                  lo = Eigen::VectorXd::Constant(variables, -1e5),
                  hi = Eigen::VectorXd::Constant(variables, 1e5);
  Eigen::VectorXd lower = Eigen::VectorXd::Constant(constraints, -1e5),
                  upper = Eigen::VectorXd::Zero(constraints);
  // Eliminate swing-foot variables rather than fixing them to zero and adding
  // redundant friction faces at the same point (degenerate active-set QPs).
  for (size_t column = 0; column < active.size(); ++column) {
    const auto foot = active[column];
    Eigen::Matrix<double, 6, Eigen::Dynamic> jac(6, m.nv);
    jac.setZero();
    pinocchio::getFrameJacobian(
        m, d, m.getFrameId(std::string(kFootFrameNames[foot])),
        pinocchio::LOCAL_WORLD_ALIGNED, jac);
    Eigen::Matrix3d cross_normal;
    cross_normal << 0, -1, 0, 1, 0, 0, 0, 0, 0;
    // Horizontal spherical contact: apply force at the material contact point,
    // not at the centre of the foot. Radius zero preserves legacy point feet.
    const Eigen::MatrixXd contact_jac =
        jac.topRows<3>() + sole_radius * cross_normal * jac.bottomRows<3>();
    A.block(0, 3 * column, 18, 3) = contact_jac.transpose();
    lo(3 * column + 2) = 0;
    // Inner pyramid conservatively satisfies ||Ft|| <= mu * Fn.
    for (int face = 0; face < 4; ++face) {
      A(18 + 4 * column + face, 3 * column + face / 2) = face % 2 ? 1 : -1;
      A(18 + 4 * column + face, 3 * column + 2) = -friction / std::sqrt(2.);
    }
  }
  lower.head<6>() = upper.head<6>() = gravity.head<6>();
  lower.segment<12>(6) = gravity.tail<12>() - m.effortLimit.tail<12>();
  upper.segment<12>(6) = gravity.tail<12>() + m.effortLimit.tail<12>();
  qpOASES::QProblem problem(variables, constraints);
  qpOASES::Options options;
  options.setToReliable();
  options.printLevel = qpOASES::PL_NONE;
  options.enableEqualities = qpOASES::BT_TRUE;
  problem.setOptions(options);
  int iterations = 200;
  const auto status =
      problem.init(H.data(), g.data(), A.data(), lo.data(), hi.data(),
                   lower.data(), upper.data(), iterations);
  out.solver_return_code = static_cast<int>(status);
  out.infeasible = problem.isInfeasible() == qpOASES::BT_TRUE;
  Eigen::VectorXd force = Eigen::VectorXd::Zero(variables);
  if (status != qpOASES::SUCCESSFUL_RETURN ||
      problem.getPrimalSolution(force.data()) != qpOASES::SUCCESSFUL_RETURN)
    return out;
  for (size_t column = 0; column < active.size(); ++column)
    out.forces.segment<3>(3 * active[column]) = force.segment<3>(3 * column);
  const Eigen::VectorXd constraint = A * force;
  out.residual = std::max(
      {(constraint - upper).maxCoeff(), (lower - constraint).maxCoeff(),
       (force - hi).maxCoeff(), (lo - force).maxCoeff(), 0.});
  out.feasible = out.forces.allFinite() && out.residual < 1e-5;
  return out;
}
} // namespace custom_dog_control
