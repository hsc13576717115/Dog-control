#include "custom_dog_control/precision/PrecisionModel.hpp"
#include <Eigen/Cholesky>
#include <coal/collision.h>
#include <coal/shape/geometric_shapes.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
namespace custom_dog_control {
struct PrecisionModel::Impl {
  RobotModelConfig model_config;
  PrecisionConfig config;
  ocs2::PinocchioInterface pin;
  ocs2::CentroidalModelInfo info;
  pinocchio::GeometryModel geom;
  std::unique_ptr<pinocchio::GeometryData> geom_data;
  std::array<int, 12> slots{};
  std::array<Eigen::Vector3d, 4> feet{}, vel{}, force{};
  Eigen::VectorXd q, v, last_v, acc;
  Impl(const ocs2::PinocchioInterface &p, const ocs2::CentroidalModelInfo &i,
       const std::string &urdf, RobotModelConfig mc, PrecisionConfig c)
      : model_config(mc), config(c), pin(p), info(i) {
    const auto &m = pin.getModel();
    q = Eigen::VectorXd::Zero(m.nq);
    v = last_v = acc = Eigen::VectorXd::Zero(m.nv);
    for (size_t j = 0; j < 12; ++j)
      slots[j] =
          m.joints[m.getJointId(std::string(kJointNames[j]))].idx_q() - 6;
    pinocchio::urdf::buildGeom(m, urdf, pinocchio::COLLISION, geom);
    geom_data = std::make_unique<pinocchio::GeometryData>(geom);
  }
};
PrecisionModel::PrecisionModel(const ocs2::PinocchioInterface &p,
                               const ocs2::CentroidalModelInfo &i,
                               const std::string &u, RobotModelConfig mc,
                               PrecisionConfig c)
    : impl_(std::make_unique<Impl>(p, i, u, mc, c)) {}
PrecisionModel::~PrecisionModel() = default;
void PrecisionModel::Reset() {
  impl_->last_v.setZero();
  impl_->acc.setZero();
}
int PrecisionModel::slot(size_t i) const { return impl_->slots[i]; }
const std::array<Eigen::Vector3d, 4> &PrecisionModel::feet() const {
  return impl_->feet;
}
const std::array<Eigen::Vector3d, 4> &PrecisionModel::velocities() const {
  return impl_->vel;
}
const std::array<Eigen::Vector3d, 4> &PrecisionModel::forces() const {
  return impl_->force;
}
Eigen::VectorXd PrecisionModel::Rbd(const JointSample &j,
                                    const EstimatedState &s) const {
  Eigen::VectorXd r = Eigen::VectorXd::Zero(36);
  r.head<3>() = s.euler_zyx;
  r.segment<3>(3) = s.position;
  r.segment<3>(18) = s.angular_velocity_world;
  r.segment<3>(21) = s.velocity_world;
  for (size_t k = 0; k < 12; ++k) {
    r(6 + slot(k)) = j.position[k];
    r(24 + slot(k)) = j.velocity[k];
  }
  return r;
}
void PrecisionModel::Measure(const JointSample &joints, const EstimatedState &s,
                             double dt) {
  auto &a = *impl_;
  const auto &m = a.pin.getModel();
  auto &d = a.pin.getData();
  a.q.head<3>() = s.position;
  a.q.segment<3>(3) = s.euler_zyx;
  a.v.head<3>() = s.velocity_world;
  a.v.segment<3>(3) =
      ocs2::getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<double>(
          s.euler_zyx, s.angular_velocity_world);
  for (size_t k = 0; k < 12; ++k) {
    a.q(6 + slot(k)) = joints.position[k];
    a.v(6 + slot(k)) = joints.velocity[k];
  }
  if (dt > 0) {
    a.acc += ((a.v - a.last_v) / std::max(dt, .0001) - a.acc) *
             std::min(dt / a.config.acc_filter_s, 1.);
    a.last_v = a.v;
  }
  a.acc = a.acc.cwiseMax(-a.config.acc_limit).cwiseMin(a.config.acc_limit);
  const Eigen::VectorXd inverse = pinocchio::rnea(m, d, a.q, a.v, a.acc);
  pinocchio::computeJointJacobians(m, d, a.q);
  pinocchio::forwardKinematics(m, d, a.q, a.v);
  pinocchio::updateFramePlacements(m, d);
  for (size_t f = 0; f < 4; ++f) {
    auto id = m.getFrameId(std::string(kFootFrameNames[f]));
    a.feet[f] = d.oMf[id].translation();
    a.vel[f] =
        pinocchio::getFrameVelocity(m, d, id, pinocchio::LOCAL_WORLD_ALIGNED)
            .linear();
    Eigen::Matrix<double, 6, Eigen::Dynamic> jac(6, m.nv);
    jac.setZero();
    pinocchio::getFrameJacobian(m, d, id, pinocchio::LOCAL_WORLD_ALIGNED, jac);
    Eigen::Matrix3d jt;
    Eigen::Vector3d residual;
    for (size_t k = 0; k < 3; ++k) {
      const int c = 6 + slot(3 * f + k);
      jt.row(k) = jac.block<3, 1>(0, c).transpose();
      residual(k) = inverse(c) - joints.effort[3 * f + k];
    }
    a.force[f] = (jt.transpose() * jt + 1e-5 * Eigen::Matrix3d::Identity())
                     .ldlt()
                     .solve(jt.transpose() * residual);
  }
}
bool PrecisionModel::Inverse(const WholeBodyReference &ref,
                             const JointSample &seed, Eigen::VectorXd &q,
                             Eigen::VectorXd &dq) {
  auto &a = *impl_;
  const auto &m = a.pin.getModel();
  auto &d = a.pin.getData();
  q = Eigen::VectorXd::Zero(m.nq);
  dq = Eigen::VectorXd::Zero(m.nv);
  q.head<3>() = ref.body;
  q.segment<3>(3) = ref.euler;
  dq.head<3>() = ref.body_velocity;
  for (size_t k = 0; k < 12; ++k)
    q(6 + slot(k)) = seed.position[k];
  for (int iter = 0; iter < 40; ++iter) {
    pinocchio::computeJointJacobians(m, d, q);
    pinocchio::updateFramePlacements(m, d);
    double error = 0;
    for (size_t f = 0; f < 4; ++f) {
      auto id = m.getFrameId(std::string(kFootFrameNames[f]));
      Eigen::Vector3d e = ref.foot[f] - d.oMf[id].translation();
      error = std::max(error, e.norm());
      Eigen::Matrix<double, 6, Eigen::Dynamic> jac(6, m.nv);
      jac.setZero();
      pinocchio::getFrameJacobian(m, d, id, pinocchio::LOCAL_WORLD_ALIGNED,
                                  jac);
      Eigen::Matrix3d j;
      for (int k = 0; k < 3; ++k)
        j.col(k) = jac.block<3, 1>(0, 6 + slot(3 * f + k));
      Eigen::Vector3d delta =
          j.transpose() *
          (j * j.transpose() + 1e-5 * Eigen::Matrix3d::Identity())
              .ldlt()
              .solve(e);
      for (int k = 0; k < 3; ++k) {
        int c = 6 + slot(3 * f + k);
        q(c) = std::clamp(q(c) + std::clamp(delta(k), -a.config.ik_delta_rad,
                                            a.config.ik_delta_rad),
                          m.lowerPositionLimit(c) + a.config.joint_margin_rad,
                          m.upperPositionLimit(c) - a.config.joint_margin_rad);
      }
      Eigen::Vector3d speed =
          j.transpose() *
          (j * j.transpose() + 1e-5 * Eigen::Matrix3d::Identity())
              .ldlt()
              .solve(ref.velocity[f] - ref.body_velocity);
      for (int k = 0; k < 3; ++k)
        dq(6 + slot(3 * f + k)) = speed(k);
    }
    if (error < a.config.ik_tolerance_m)
      return q.allFinite() && dq.allFinite();
  }
  return false;
}
bool PrecisionModel::ApplyProbe(int foot, double force,
                                HybridJointCommand &command) {
  auto &a = *impl_;
  const auto &m = a.pin.getModel();
  auto &d = a.pin.getData();
  if (foot >= 0 && foot < 4) {
    pinocchio::computeJointJacobians(m, d, a.q);
    pinocchio::updateFramePlacements(m, d);
    Eigen::Matrix<double, 6, Eigen::Dynamic> jac(6, m.nv);
    jac.setZero();
    pinocchio::getFrameJacobian(
        m, d, m.getFrameId(std::string(kFootFrameNames[foot])),
        pinocchio::LOCAL_WORLD_ALIGNED, jac);
    for (size_t k = 0; k < 3; ++k)
      command.effort[3 * foot + k] -=
          jac(2, 6 + slot(3 * foot + k)) *
          std::clamp(force, 0., a.config.probe_force_n);
  }
  for (size_t k = 0; k < 12; ++k)
    if (!std::isfinite(command.effort[k]) ||
        std::abs(command.effort[k]) > m.effortLimit(6 + slot(k)))
      return false;
  return true;
}
bool PrecisionModel::CollisionFree(const Eigen::VectorXd &q,
                                   const Eigen::Vector3d &center, double height,
                                   const Eigen::Vector2d &size) {
  auto &a = *impl_;
  auto &gd = *a.geom_data;
  pinocchio::updateGeometryPlacements(a.pin.getModel(), a.pin.getData(), a.geom,
                                      gd, q);
  coal::Box pad(size.x(), size.y(), std::max(height, .0001));
  coal::Transform3s tf(Eigen::Matrix3d::Identity(),
                       Eigen::Vector3d(center.x(), center.y(), height / 2));
  for (size_t i = 0; i < a.geom.geometryObjects.size(); ++i) {
    const auto &g = a.geom.geometryObjects[i];
    const auto &pose = gd.oMg[i];
    coal::CollisionRequest req;
    coal::CollisionResult res;
    const auto &link = a.pin.getModel().frames[g.parentFrame].name;
    // Sole contact with the planned support is allowed; all leg/body shapes
    // remain checked.
    const bool is_foot = link.find("_foot") != std::string::npos;
    coal::collide(g.geometry.get(),
                  coal::Transform3s(pose.rotation(), pose.translation()), &pad,
                  tf, req, res);
    if (res.isCollision() &&
        (!is_foot || pose.translation().z() < height +
                                                  a.model_config.foot_radius_m -
                                                  a.config.plane_tolerance_m))
      return false;
  }
  return true;
}
Eigen::Vector3d PrecisionModel::CenterOfMass(const Eigen::VectorXd &q) {
  return pinocchio::centerOfMass(impl_->pin.getModel(), impl_->pin.getData(),
                                 q);
}
} // namespace custom_dog_control
