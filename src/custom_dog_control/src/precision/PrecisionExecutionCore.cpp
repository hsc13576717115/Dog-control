#include "custom_dog_control/precision/PrecisionExecutionCore.hpp"
#include "custom_dog_control/control/SampleFreshness.hpp"
#include "custom_dog_control/precision/ContactObserver.hpp"
#include "custom_dog_control/precision/FootstepExecutor.hpp"
#include "custom_dog_control/precision/PrecisionModel.hpp"
#include "custom_dog_control/precision/PrecisionWbc.hpp"
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
namespace custom_dog_control {
struct PrecisionExecutionCore::Impl {
  PrecisionExecutionCore::Clock clock;
  SampleFreshness joint_freshness, imu_freshness;
  RobotModelConfig model_config;
  PrecisionConfig config;
  ocs2::CentroidalModelInfo info;
  PrecisionModel model;
  KinematicStateEstimator estimator;
  std::unique_ptr<PrecisionWbc> wbc;
  FootstepExecutor executor;
  std::array<ContactObserver, 4> observers;
  std::array<ContactEstimate, 4> contacts{};
  ContactSupport support;
  EstimatedState state;
  WholeBodyReference reference;
  JointSample initial;
  PrecisionSnapshot latest;
  bool initialized = false, tracking = false, fault = false;
  double activated = -1;
  uint64_t consumed = 0;
  Impl(const RobotModel &robot, const std::string &urdf,
       const std::string &task, const RobotModelConfig &m,
       const PrecisionConfig &c, PrecisionExecutionCore::Clock clock_fn)
      : clock(clock_fn), model_config(m), config(c), info(robot.info),
        model(robot.pin, robot.info, urdf, m, c),
        estimator(robot.pin, robot.info), executor(c) {
    if (!clock)
      throw std::invalid_argument("missing monotonic clock");
    std::vector<std::string> names;
    for (auto s : kFootFrameNames)
      names.emplace_back(s);
    ocs2::CentroidalModelPinocchioMapping mapping(info);
    ocs2::PinocchioEndEffectorKinematics ee(robot.pin, mapping, names);
    wbc = std::make_unique<PrecisionWbc>(robot.pin, info, ee);
    wbc->Configure(config, model_config.foot_radius_m);
    wbc->loadTasksSetting(task, false);
    Reset();
  }
  void Reset() {
    joint_freshness = {};
    imu_freshness = {};
    initialized = tracking = fault = false;
    activated = -1;
    consumed = 0;
    contacts = {};
    support = {};
    reference = {};
    state = {};
    latest = {};
    for (auto &observer : observers)
      observer = ContactObserver(config);
    support.foot_center_height.fill(model_config.foot_radius_m +
                                   model_config.initial_support_height_m);
    support.height_valid.fill(true);
    state.position.z() = model_config.initial_height_m;
    estimator.Reset(model_config.initial_height_m);
    model.Reset();
    executor = FootstepExecutor(config);
  }
  bool Update(double now, double dt, const JointSample &joints,
              const ImuSample &imu, bool estop, const PrecisionCommand &command,
              bool cancel, HybridJointCommand &output) {
    if (!initialized) {
      initial = joints;
      activated = now;
      initialized = true;
    }
    bool joints_valid = joint_freshness.Accept(
        now, joints.stamp_seconds, joints.stamp_valid, config.state_timeout_s);
    for (size_t k = 0; k < 12; ++k)
      joints_valid = joints_valid && joints.valid[k] > .5 &&
                     std::isfinite(joints.position[k]) &&
                     std::isfinite(joints.velocity[k]) &&
                     std::isfinite(joints.effort[k]);
    bool valid =
        joints_valid && imu_freshness.Accept(now, imu.stamp_seconds, imu.valid,
                                             config.state_timeout_s);
    for (size_t k = 0; k < 3; ++k)
      valid = valid && std::isfinite(imu.angular_velocity[k]) &&
              std::isfinite(imu.linear_acceleration[k]);
    if (!valid && joints_valid && !fault && !estop &&
        now - activated < config.startup_grace_s) {
      // Simulation starts in a standing pose. Hold it while the first IMU
      // message arrives; damping-only output would let the unsupported joints
      // collapse before state estimation starts. No state-ready claim is made.
      for (size_t k = 0; k < 12; ++k) {
        output.position[k] = initial.position[k];
        output.kp[k] = config.startup_kp;
        output.kd[k] = config.startup_kd;
      }
      return true;
    }
    if (fault) {
      Save(now, joints, 0, 0);
      return false;
    }
    if (estop || !valid) {
      fault = true;
      executor.Abort(estop ? PrecisionError::ESTOP
                           : PrecisionError::INVALID_STATE);
      Save(now, joints, 0, 0);
      return false;
    }
    Eigen::Quaterniond rot(imu.orientation_wxyz[0], imu.orientation_wxyz[1],
                           imu.orientation_wxyz[2], imu.orientation_wxyz[3]);
    if (!std::isfinite(rot.norm()) || rot.norm() < 1e-6) {
      fault = true;
      executor.Abort(PrecisionError::INVALID_ORIENTATION);
      Save(now, joints, 0, 0);
      return false;
    }
    rot.normalize();
    state.euler_zyx = EulerZyxFromRotation(rot.toRotationMatrix());
    state.angular_velocity_world =
        rot * Eigen::Vector3d(imu.angular_velocity.data());
    model.Measure(joints, state, dt);
    for (size_t f = 0; f < 4; ++f) {
      // For a sphere on a known horizontal support, v_center = r * omega x n.
      // This uses IMU + joint kinematics only, not simulator contact witnesses.
      const Eigen::Vector3d rolling =
          config.sole_rolling_model > .5
              ? (model_config.foot_radius_m *
                 model.angularVelocities()[f].cross(Eigen::Vector3d::UnitZ()))
                    .eval()
              : Eigen::Vector3d::Zero();
      contacts[f] = observers[f].Update(now, model.forces()[f],
                                        model.velocities()[f] - rolling, valid);
      support.contact[f] = contacts[f].loaded && !contacts[f].slipping;
      if (tracking) {
        // Planned swing can exclude a foot from the estimator, but never
        // declare a planned stance to be an observed contact.
        support.contact[f] = support.contact[f] && reference.contact[f];
        if (support.contact[f]) {
          if (!support.anchor_valid[f])
            support.anchor_position[f] = model.feet()[f];
          else
            support.anchor_position[f] += dt * rolling;
        }
        support.anchor_valid[f] = support.contact[f];
        support.height_valid[f] = reference.contact[f] && support.contact[f] &&
                                  (model.feet()[f] - reference.foot[f]).norm() <
                                      config.target_tolerance_m;
        if (support.height_valid[f])
          support.foot_center_height[f] = reference.foot[f].z();
      }
      support.center_velocity[f] =
          support.contact[f] ? rolling : Eigen::Vector3d::Zero();
    }
    state = estimator.Update(joints, imu, support, dt);
    model.Measure(
        joints, state,
        0); // FK must correspond to the updated continuous base estimate.
    state.foot_position_world = model.feet();
    if (!state.valid ||
        std::abs(state.euler_zyx.y()) > config.attitude_limit_rad ||
        std::abs(state.euler_zyx.z()) > config.attitude_limit_rad) {
      fault = true;
      executor.Abort(PrecisionError::STATE_OR_ATTITUDE);
      Save(now, joints, 0, 0);
      return false;
    }
    if (!tracking) {
      for (size_t k = 0; k < 12; ++k) {
        output.position[k] = initial.position[k];
        output.kp[k] = config.startup_kp;
        output.kd[k] = config.startup_kd;
      }
      bool loaded = true;
      for (auto c : contacts)
        loaded = loaded && c.loaded;
      if (now - activated > config.startup_settle_s && loaded) {
        reference.body = state.position;
        reference.euler = state.euler_zyx;
        reference.foot = model.feet();
        support.anchor_position = reference.foot;
        support.anchor_valid.fill(true);
        executor.Reset(reference);
        tracking = true;
      }
      if (now - activated > config.startup_timeout_s && !tracking) {
        fault = true;
        executor.Abort(PrecisionError::INITIAL_SUPPORT_UNCONFIRMED);
      }
      Save(now, joints, 0, 0);
      return !fault;
    }
    const auto &p = command;
    if (p.valid && p.step.id != consumed) {
      consumed = p.step.id;
      executor.Start(p.step, now);
    }
    if (cancel)
      executor.Abort(PrecisionError::CANCELED);
    executor.RollSupports(support.center_velocity, support.contact, dt);
    reference = executor.Update(now, contacts, model.feet());
    if (executor.error_code() == PrecisionError::SUPPORT_UNCONFIRMED ||
        executor.error_code() == PrecisionError::TOUCHDOWN_LOST) {
      // An unsupported HOLD cannot safely reuse the previous contact-force QP.
      fault = true;
      Save(now, joints, 0, 0);
      return false;
    }
    for (size_t f = 0; f < 4; ++f)
      if (reference.contact[f] && contacts[f].loaded &&
          (model.feet()[f] - reference.foot[f]).norm() <
              config.target_tolerance_m)
        support.foot_center_height[f] = reference.foot[f].z();
    Eigen::VectorXd q, dq;
    if (!model.Inverse(reference, joints, q, dq)) {
      executor.Abort(PrecisionError::RUNTIME_IK);
      fault = true;
      Save(now, joints, 0, 0);
      return false;
    }
    Eigen::VectorXd desired = Eigen::VectorXd::Zero(24),
                    input = Eigen::VectorXd::Zero(24);
    desired.segment<6>(6) = q.head<6>();
    desired.tail<12>() = q.tail<12>();
    input.tail<12>() = dq.tail<12>();
    int nc = 0;
    for (bool c : reference.contact)
      nc += c;
    for (size_t f = 0; f < 4; ++f)
      if (reference.contact[f])
        input(3 * f + 2) = info.robotMass * 9.81 / std::max(nc, 1);
    wbc->Reference(reference);
    const double begin = clock();
    auto result = wbc->update(desired, input, model.Rbd(joints, state),
                              ContactMode(reference.contact), dt);
    const double ms = 1000. * (clock() - begin);
    if (!std::isfinite(ms) || ms < 0 || !wbc->lastSolverSucceeded() ||
        wbc->lastEqualityResidual() > config.residual_limit ||
        wbc->lastInequalityViolation() > config.residual_limit ||
        ms > config.solve_timeout_ms) {
      executor.Abort(PrecisionError::WBC_INVALID_OR_TIMEOUT);
      fault = true;
      Save(now, joints, ms, wbc->lastEqualityResidual());
      return false;
    }
    for (size_t k = 0; k < 12; ++k) {
      int idx = model.slot(k);
      output.position[k] = q(6 + idx);
      output.velocity[k] = dq(6 + idx);
      output.effort[k] = result(30 + idx);
      output.kp[k] = 0;
      output.kd[k] = 0;
    }
    if (!model.ApplyProbe(reference.probe_foot, reference.probe_force,
                          output)) {
      executor.Abort(PrecisionError::TOTAL_TORQUE_LIMIT);
      fault = true;
      Save(now, joints, ms, wbc->lastEqualityResidual());
      return false;
    }
    Save(now, joints, ms, wbc->lastEqualityResidual());
    return true;
  }
  void Save(double now, const JointSample &joints, double ms, double residual) {
    PrecisionSnapshot s;
    s.now = now;
    s.steady_stamp = PrecisionSteadyNow();
    s.joints = joints;
    s.estimate = state;
    s.estimate.valid = state.valid && !fault;
    s.ref = executor.reference();
    s.contact = contacts;
    s.phase = executor.phase();
    s.id = executor.id();
    s.error = executor.error_code();
    s.solve_ms = ms;
    s.residual = residual;
    s.ready = tracking && !fault &&
              (s.phase == StepPhase::STANCE || s.phase == StepPhase::DONE);
    for (auto c : contacts)
      s.ready = s.ready && c.loaded && !c.slipping;
    latest = s;
  }
};
PrecisionExecutionCore::PrecisionExecutionCore(
    const RobotModel &r, const std::string &u, const std::string &t,
    const RobotModelConfig &m, const PrecisionConfig &c, Clock clock)
    : impl_(std::make_unique<Impl>(r, u, t, m, c, clock)) {}
PrecisionExecutionCore::~PrecisionExecutionCore() = default;
void PrecisionExecutionCore::Reset() { impl_->Reset(); }
bool PrecisionExecutionCore::Update(double n, double dt, const JointSample &j,
                                    const ImuSample &i, bool stop,
                                    const PrecisionCommand &p, bool cancel,
                                    HybridJointCommand &out) {
  return impl_->Update(n, dt, j, i, stop, p, cancel, out);
}
const PrecisionSnapshot &PrecisionExecutionCore::snapshot() const {
  return impl_->latest;
}
} // namespace custom_dog_control
