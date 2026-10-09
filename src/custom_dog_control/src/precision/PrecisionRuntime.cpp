#include "custom_dog_control/precision/PrecisionRuntime.hpp"
#include "custom_dog_control/precision/ContactObserver.hpp"
#include "custom_dog_control/precision/LatestSnapshot.hpp"
#include "custom_dog_control/precision/FootstepExecutor.hpp"
#include "custom_dog_control/precision/PrecisionModel.hpp"
#include "custom_dog_control/precision/PrecisionWbc.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <qr_interfaces/action/execute_footsteps.hpp>
#include <qr_interfaces/msg/foot_contact_array.hpp>
#include <qr_interfaces/msg/robot_state.hpp>
#include <qr_interfaces/msg/support_region_array.hpp>
#include <qr_interfaces/srv/plan_footsteps.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <realtime_tools/realtime_buffer.hpp>
namespace custom_dog_control {
namespace {
using Action = qr_interfaces::action::ExecuteFootsteps;
using Goal = rclcpp_action::ServerGoalHandle<Action>;
Eigen::Vector3d Vec(const geometry_msgs::msg::Point &p) {
  return {p.x, p.y, p.z};
}
geometry_msgs::msg::Point Point(const Eigen::Vector3d &v) {
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}
std::size_t Mode(const std::array<bool, 4> &flags) {
  std::size_t m = 0;
  for (size_t i = 0; i < 4; ++i)
    if (flags[i])
      m |= (1U << (3 - i));
  return m;
}
struct Snapshot {
  JointSample joints;
  EstimatedState estimate;
  WholeBodyReference ref;
  std::array<ContactEstimate, 4> contact{};
  StepPhase phase = StepPhase::STANCE;
  uint64_t id = 0;
  double now = 0, solve_ms = 0, residual = 0;
  bool ready = false;
  std::string error;
};
struct Pending {
  PrecisionStep step;
  bool valid = false;
};
} // namespace
struct PrecisionRuntime::Impl {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  ocs2::PinocchioInterface pin;
  ocs2::CentroidalModelInfo info;
  PrecisionModel model, preflight;
  KinematicStateEstimator estimator;
  std::unique_ptr<PrecisionWbc> wbc, check_wbc;
  FootstepExecutor executor;
  std::array<ContactObserver, 4> observers;
  std::array<ContactEstimate, 4> contacts{};
  ContactSupport support;
  EstimatedState state;
  WholeBodyReference reference;
  JointSample initial;
  LatestSnapshot<Snapshot> snapshot;
  realtime_tools::RealtimeBuffer<Pending> pending;
  std::atomic<bool> active{false}, cancel{false}, busy{false};
  bool initialized = false, tracking = false, fault = false;
  double activated = -1;
  uint64_t consumed = 0, next_id = 1, accepted_id = 0;
  std::mutex callback_mutex;
  qr_interfaces::msg::SupportRegionArray regions;
  qr_interfaces::msg::FootstepPlan approved;
  PrecisionStep approved_step;
  WholeBodyReference approved_reference;
  std::shared_ptr<Goal> goal;
  rclcpp::Service<qr_interfaces::srv::PlanFootsteps>::SharedPtr planner;
  rclcpp_action::Server<Action>::SharedPtr action;
  rclcpp::Subscription<qr_interfaces::msg::SupportRegionArray>::SharedPtr
      region_sub;
  rclcpp_lifecycle::LifecyclePublisher<
      qr_interfaces::msg::RobotState>::SharedPtr state_pub;
  rclcpp_lifecycle::LifecyclePublisher<
      qr_interfaces::msg::ExecutionStatus>::SharedPtr status_pub;
  rclcpp_lifecycle::LifecyclePublisher<
      qr_interfaces::msg::FootContactArray>::SharedPtr contact_pub;
  rclcpp::TimerBase::SharedPtr timer;
  Impl(rclcpp_lifecycle::LifecycleNode::SharedPtr n, const NmpcBackend &backend,
       const std::string &urdf, const std::string &task)
      : node(n), pin(backend.pinocchioInterface()), info(backend.modelInfo()),
        model(pin, info, urdf), preflight(pin, info, urdf),
        estimator(pin, info) {
    std::vector<std::string> names;
    for (auto s : kFootFrameNames)
      names.emplace_back(s);
    ocs2::CentroidalModelPinocchioMapping mapping(info);
    ocs2::PinocchioEndEffectorKinematics ee(pin, mapping, names);
    wbc = std::make_unique<PrecisionWbc>(pin, info, ee);
    check_wbc = std::make_unique<PrecisionWbc>(pin, info, ee);
    wbc->loadTasksSetting(task, false);
    check_wbc->loadTasksSetting(task, false);
    estimator.Reset(.29);
    state.position.z() = .29;
    support.foot_center_height.fill(.026);
    support.height_valid.fill(true);
    state_pub =
        node->create_publisher<qr_interfaces::msg::RobotState>("/qr/state", 10);
    status_pub = node->create_publisher<qr_interfaces::msg::ExecutionStatus>(
        "/qr/execution", 10);
    contact_pub = node->create_publisher<qr_interfaces::msg::FootContactArray>(
        "/qr/contact_estimates", 10);
    region_sub =
        node->create_subscription<qr_interfaces::msg::SupportRegionArray>(
            "/qr/support_regions", rclcpp::QoS(1).transient_local(),
            [this](qr_interfaces::msg::SupportRegionArray::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(callback_mutex);
              regions = *msg;
            });
    planner = node->create_service<qr_interfaces::srv::PlanFootsteps>(
        "/qr/plan_footsteps",
        [this](qr_interfaces::srv::PlanFootsteps::Request::SharedPtr req,
               qr_interfaces::srv::PlanFootsteps::Response::SharedPtr res) {
          std::lock_guard<std::mutex> lock(callback_mutex);
          Plan(*req, *res);
        });
    action = rclcpp_action::create_server<Action>(
        node, "/qr/execute_footsteps",
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const Action::Goal> request) {
          std::lock_guard<std::mutex> lock(callback_mutex);
          auto s = snapshot.Read();
          if (!active || busy || !s.ready || request->plan != approved ||
              request->plan.id == accepted_id ||
              regions.revision != approved.map_revision ||
              node->now().seconds() - s.now > .1 ||
              approved.steps.size() != 1 ||
              node->now() > rclcpp::Time(approved.valid_until))
            return rclcpp_action::GoalResponse::REJECT;
          if ((s.ref.body - approved_reference.body).norm() > .005)
            return rclcpp_action::GoalResponse::REJECT;
          for (size_t f = 0; f < 4; ++f)
            if ((s.estimate.foot_position_world[f] - approved_reference.foot[f])
                    .norm() > .02)
              return rclcpp_action::GoalResponse::REJECT;
          accepted_id = approved.id;
          cancel = false;
          busy = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<Goal>) {
          cancel = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<Goal> g) {
          std::lock_guard<std::mutex> lock(callback_mutex);
          goal = g;
          pending.writeFromNonRT(Pending{approved_step, true});
        });
    timer = node->create_wall_timer(std::chrono::milliseconds(20),
                                    [this] { Publish(); });
  }
  void Plan(const qr_interfaces::srv::PlanFootsteps::Request &req,
            qr_interfaces::srv::PlanFootsteps::Response &res) {
    auto s = snapshot.Read();
    res.accepted = false;
    auto reject = [&](const char *reason) { res.reason = reason; };
    if (!active || busy || !s.ready || node->now().seconds() - s.now > .1) {
      reject("state_not_ready");
      return;
    }
    if (req.foot >= 4 || regions.header.frame_id != "odom") {
      reject("invalid_foot_or_frame");
      return;
    }
    const auto it =
        std::find_if(regions.regions.begin(), regions.regions.end(),
                     [&](const auto &p) { return p.id == req.surface_id; });
    if (it == regions.regions.end() || !it->known || it->forbidden ||
        !std::isfinite(it->confidence) || it->confidence < .9 ||
        it->polygon.points.size() != 4) {
      reject("unknown_surface");
      return;
    }
    const double age =
        node->now().seconds() - rclcpp::Time(it->observed_at).seconds();
    if (age < 0 || age > .5) {
      reject("stale_surface");
      return;
    }
    if (!std::isfinite(it->normal.z) || !std::isfinite(it->normal.x) ||
        !std::isfinite(it->normal.y) ||
        std::abs(it->normal.x) + std::abs(it->normal.y) > 1e-3 ||
        std::abs(it->normal.z - 1.) > 1e-3) {
      reject("M1_horizontal_surfaces_only");
      return;
    }
    for (const auto &point : it->polygon.points) {
      if (!std::isfinite(point.z) ||
          std::abs(point.z - it->polygon.points.front().z) > 1e-5) {
        reject("invalid_plane");
        return;
      }
    }
    Eigen::Vector3d target = Vec(req.target);
    std::vector<Eigen::Vector2d> poly;
    for (auto p : it->polygon.points)
      poly.push_back({p.x, p.y});
    if (!target.allFinite() ||
        !qr_planning::Inside(poly, target.head<2>(), .046) ||
        std::abs(target.z() - it->polygon.points[0].z - .026) > .002) {
      reject("outside_eroded_surface");
      return;
    }
    if ((target - s.ref.foot[req.foot]).norm() > .25 || target.z() < .025 ||
        target.z() > .077) {
      reject("M1_motion_envelope");
      return;
    }
    // M1 accepts only axis-aligned horizontal rectangular fixtures. Check all
    // supplied pads, not a fictitious pad centered at the requested footpoint.
    struct Box {
      Eigen::Vector3d center;
      Eigen::Vector2d size;
      double height;
    };
    std::vector<Box> boxes{{Eigen::Vector3d::Zero(), {10., 10.}, 0.}};
    for (const auto &surface : regions.regions) {
      const double surface_age =
          node->now().seconds() - rclcpp::Time(surface.observed_at).seconds();
      if (!surface.known || surface.forbidden || surface.confidence < .9 ||
          !std::isfinite(surface.confidence) || surface_age < 0 ||
          surface_age > .5 || surface.polygon.points.size() != 4) {
        reject("M1_incomplete_fixture_geometry");
        return;
      }
      const auto &points = surface.polygon.points;
      const Eigen::Vector3d normal(surface.normal.x, surface.normal.y,
                                   surface.normal.z);
      Eigen::Vector2d lo(points[0].x, points[0].y), hi = lo;
      for (const auto &p : points) {
        Eigen::Vector3d xyz(p.x, p.y, p.z);
        if (!xyz.allFinite() || std::abs(p.z - points[0].z) > 1e-5) {
          reject("M1_invalid_fixture_geometry");
          return;
        }
        lo = lo.cwiseMin(xyz.head<2>());
        hi = hi.cwiseMax(xyz.head<2>());
      }
      const Eigen::Vector2d size = hi - lo;
      if (!normal.allFinite() ||
          (normal - Eigen::Vector3d::UnitZ()).norm() > 1e-3 ||
          size.minCoeff() < .092 || points[0].z < 0 || points[0].z > .051) {
        reject("M1_rectangular_horizontal_fixtures_only");
        return;
      }
      const std::array<Eigen::Vector2d, 4> corners{
          lo, {hi.x(), lo.y()}, hi, {lo.x(), hi.y()}};
      for (size_t i = 0; i < 4; ++i) {
        if ((Eigen::Vector2d(points[i].x, points[i].y) - corners[i]).norm() >
            1e-5) {
          reject("M1_rectangular_horizontal_fixtures_only");
          return;
        }
      }
      boxes.push_back({{.5 * (lo.x() + hi.x()), .5 * (lo.y() + hi.y()), 0.},
                       size,
                       points[0].z});
    }
    PrecisionStep step;
    step.id = next_id++;
    step.foot = req.foot;
    step.target = target;
    Eigen::VectorXd initial_q, initial_dq;
    if (!preflight.Inverse(s.ref, s.joints, initial_q, initial_dq)) {
      reject("initial_stance_ik");
      return;
    }
    const Eigen::Vector3d com = preflight.CenterOfMass(initial_q);
    const Eigen::Vector3d support_center =
        qr_planning::SupportCentroid(s.ref.foot, req.foot);
    std::vector<Eigen::Vector2d> support_polygon;
    for (size_t f = 0; f < 4; ++f)
      if (f != req.foot)
        support_polygon.push_back(s.ref.foot[f].head<2>());
    std::sort(support_polygon.begin(), support_polygon.end(),
              [&](const auto &a, const auto &b) {
                return std::atan2(a.y() - support_center.y(),
                                  a.x() - support_center.x()) <
                       std::atan2(b.y() - support_center.y(),
                                  b.x() - support_center.x());
              });
    bool found_support = false;
    // Shift the actual model COM just far enough inside the support triangle.
    // Moving the base origin to the triangle centroid ignores the COM offset
    // and causes unnecessary leg rotation and rolling of spherical feet.
    for (int i = 0; i <= 100; ++i) {
      const double blend = i / 100.;
      const Eigen::Vector2d next_com =
          (1. - blend) * com.head<2>() + blend * support_center.head<2>();
      if (qr_planning::Inside(support_polygon, next_com, .03)) {
        step.body = s.ref.body;
        step.body.head<2>() += next_com - com.head<2>();
        found_support = true;
        break;
      }
    }
    if (!found_support) {
      reject("support_margin_infeasible");
      return;
    }
    auto validate = [&]() {
      FootstepExecutor trial;
      trial.Reset(s.ref);
      trial.Start(step, 0.);
      // Sample all reference phases before accepting the action. End-state IK
      // is included; the solver cannot hide an unreachable next stance.
      auto cs = s.contact;
      for (auto &c : cs) {
        c.valid = c.loaded = true;
        c.slipping = false;
      }
      Eigen::VectorXd q, dq;
      const double duration = 2 * step.shift + step.swing + .3;
      for (double t = 0; t < duration; t += .05) {
        cs[step.foot].loaded = t >= step.shift + step.swing;
        auto feet = s.ref.foot;
        feet[step.foot] = target;
        const auto ref = trial.Update(t, cs, feet);
        if (trial.phase() == StepPhase::HOLD) {
          reject("reference_generation_failed");
          return false;
        }
        if (!preflight.Inverse(ref, s.joints, q, dq)) {
          reject("ik_or_joint_margin");
          return false;
        }
        for (const auto &box : boxes)
          if (!preflight.CollisionFree(q, box.center, box.height, box.size)) {
            reject("leg_or_body_collision");
            return false;
          }
        Eigen::VectorXd xd = Eigen::VectorXd::Zero(24),
                        u = Eigen::VectorXd::Zero(24),
                        rbd = Eigen::VectorXd::Zero(36);
        xd.segment<6>(6) = q.head<6>();
        xd.tail<12>() = q.tail<12>();
        rbd.head<3>() = q.segment<3>(3);
        rbd.segment<3>(3) = q.head<3>();
        rbd.segment<12>(6) = q.tail<12>();
        int count = 0;
        for (bool c : ref.contact)
          count += c;
        for (size_t f = 0; f < 4; ++f)
          if (ref.contact[f])
            u(3 * f + 2) = info.robotMass * 9.81 / count;
        check_wbc->Reference(ref);
        check_wbc->update(xd, u, rbd, Mode(ref.contact), .001);
        if (!check_wbc->lastSolverSucceeded() ||
            check_wbc->lastEqualityResidual() > 1e-3 ||
            check_wbc->lastInequalityViolation() > 1e-3) {
          reject("support_or_torque_infeasible");
          return false;
        }
      }
      return true;
    };
    bool feasible = false;
    for (double lift : {0., .015, .03}) {
      step.body.z() = s.ref.body.z() + lift;
      if (validate()) {
        feasible = true;
        break;
      }
    }
    if (!feasible)
      return;
    qr_interfaces::msg::Footstep out;
    out.foot = req.foot;
    out.target = req.target;
    out.surface_id = req.surface_id;
    out.safe_region = it->polygon;
    // Publish the actual safe region, consistent with the 46 mm admission
    // margin, rather than labelling the unshrunk support surface as safe.
    out.safe_region.points[0].x += .046;
    out.safe_region.points[0].y += .046;
    out.safe_region.points[1].x -= .046;
    out.safe_region.points[1].y += .046;
    out.safe_region.points[2].x -= .046;
    out.safe_region.points[2].y -= .046;
    out.safe_region.points[3].x += .046;
    out.safe_region.points[3].y -= .046;
    out.body_target = Point(step.body);
    out.shift_duration = step.shift;
    out.swing_duration = step.swing;
    out.clearance = step.clearance;
    out.contact_timeout = step.timeout;
    approved = {};
    approved.header.frame_id = "odom";
    approved.header.stamp = node->now();
    approved.id = step.id;
    approved.map_revision = regions.revision;
    approved.valid_until = (node->now() + rclcpp::Duration::from_seconds(1.));
    approved.steps = {out};
    approved_step = step;
    approved_reference = s.ref;
    res.accepted = true;
    res.reason = "validated_single_step";
    res.plan = approved;
  }
  void Publish() {
    if (!active)
      return;
    auto s = snapshot.Read();
    if (s.now <= 0)
      return;
    qr_interfaces::msg::RobotState msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(s.now * 1e9));
    msg.pose.position = Point(s.estimate.position);
    Eigen::Quaterniond rot(
        Eigen::AngleAxisd(s.estimate.euler_zyx.x(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(s.estimate.euler_zyx.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(s.estimate.euler_zyx.z(), Eigen::Vector3d::UnitX()));
    msg.pose.orientation.x = rot.x();
    msg.pose.orientation.y = rot.y();
    msg.pose.orientation.z = rot.z();
    msg.pose.orientation.w = rot.w();
    msg.joint_position = s.joints.position;
    msg.joint_velocity = s.joints.velocity;
    msg.valid = s.estimate.valid;
    msg.twist.linear.x = s.estimate.velocity_world.x();
    msg.twist.linear.y = s.estimate.velocity_world.y();
    msg.twist.linear.z = s.estimate.velocity_world.z();
    msg.twist.angular.x = s.estimate.angular_velocity_world.x();
    msg.twist.angular.y = s.estimate.angular_velocity_world.y();
    msg.twist.angular.z = s.estimate.angular_velocity_world.z();
    // Covariance not exported by the current filter: negative diagonal means
    // unknown.
    for (size_t k = 0; k < 6; ++k)
      msg.pose_covariance[7 * k] = -1;
    qr_interfaces::msg::ExecutionStatus status;
    status.header = msg.header;
    status.plan_id = s.id;
    status.phase = PhaseName(s.phase);
    status.error = s.error;
    status.ready = s.ready;
    status.completed_steps = s.phase == StepPhase::DONE ? 1 : 0;
    status.solve_time_ms = s.solve_ms;
    status.equality_residual = s.residual;
    qr_interfaces::msg::FootContactArray contact;
    contact.header = msg.header;
    for (size_t f = 0; f < 4; ++f) {
      msg.feet[f] = Point(s.estimate.foot_position_world[f]);
      msg.contacts[f] = s.contact[f].loaded;
      status.contacts[f] = msg.contacts[f];
      status.targets[f] = Point(s.ref.foot[f]);
      auto &c = contact.feet[f];
      c.stamp = rclcpp::Time(static_cast<int64_t>(s.contact[f].stamp * 1e9));
      c.valid = s.contact[f].valid;
      c.probability = s.contact[f].probability;
      c.slipping = s.contact[f].slipping;
      c.force_valid = false;
      c.effort_estimate_valid = s.contact[f].valid;
      c.estimated_force.x = s.contact[f].estimated_force.x();
      c.estimated_force.y = s.contact[f].estimated_force.y();
      c.estimated_force.z = s.contact[f].estimated_force.z();
    }
    state_pub->publish(msg);
    status_pub->publish(status);
    contact_pub->publish(contact);
    std::lock_guard<std::mutex> lock(callback_mutex);
    if (goal && s.id == approved.id) {
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->status = status;
      goal->publish_feedback(feedback);
      if (s.phase == StepPhase::DONE || s.phase == StepPhase::HOLD) {
        auto result = std::make_shared<Action::Result>();
        result->success = s.phase == StepPhase::DONE;
        result->reason = s.error;
        result->completed_steps = status.completed_steps;
        if (goal->is_canceling())
          goal->canceled(result);
        else if (result->success)
          goal->succeed(result);
        else
          goal->abort(result);
        goal.reset();
        busy = false;
      }
    }
  }
  bool Update(double now, double dt, const JointSample &joints,
              const ImuSample &imu, bool estop, HybridJointCommand &output) {
    if (!active)
      return false;
    if (!initialized) {
      initial = joints;
      activated = now;
      initialized = true;
    }
    bool joints_valid = true;
    for (size_t k = 0; k < 12; ++k)
      joints_valid = joints_valid && joints.valid[k] > .5 &&
                     std::isfinite(joints.position[k]) &&
                     std::isfinite(joints.velocity[k]) &&
                     std::isfinite(joints.effort[k]);
    bool valid = joints_valid && imu.valid && now - imu.stamp_seconds < .1 &&
                 now >= imu.stamp_seconds;
    if (!valid && joints_valid && !fault && !estop && now - activated < .25) {
      // Simulation starts in a standing pose. Hold it while the first IMU
      // message arrives; damping-only output would let the unsupported joints
      // collapse before state estimation starts. No state-ready claim is made.
      for (size_t k = 0; k < 12; ++k) {
        output.position[k] = initial.position[k];
        output.kp[k] = 60.;
        output.kd[k] = 3.;
      }
      return true;
    }
    if (fault) {
      Save(now, joints, 0, 0);
      return false;
    }
    if (estop || !valid) {
      fault = true;
      executor.Abort(estop ? "estop" : "invalid_state");
      Save(now, joints, 0, 0);
      return false;
    }
    Eigen::Quaterniond rot(imu.orientation_wxyz[0], imu.orientation_wxyz[1],
                           imu.orientation_wxyz[2], imu.orientation_wxyz[3]);
    if (!std::isfinite(rot.norm()) || rot.norm() < 1e-6) {
      fault = true;
      executor.Abort("invalid_orientation");
      Save(now, joints, 0, 0);
      return false;
    }
    rot.normalize();
    state.euler_zyx = EulerZyxFromRotation(rot.toRotationMatrix());
    state.angular_velocity_world =
        rot * Eigen::Vector3d(imu.angular_velocity.data());
    model.Measure(joints, state, dt);
    for (size_t f = 0; f < 4; ++f) {
      contacts[f] = observers[f].Update(now, model.forces()[f],
                                        model.velocities()[f], valid);
      support.contact[f] = contacts[f].loaded && !contacts[f].slipping;
      if (tracking) {
        // Planned swing can exclude a foot from the estimator, but never
        // declare a planned stance to be an observed contact.
        support.contact[f] = support.contact[f] && reference.contact[f];
        if (support.contact[f] && !support.anchor_valid[f])
          support.anchor_position[f] = model.feet()[f];
        support.anchor_valid[f] = support.contact[f];
        support.height_valid[f] =
            reference.contact[f] && support.contact[f] &&
            (model.feet()[f] - reference.foot[f]).norm() < .02;
        if (support.height_valid[f])
          support.foot_center_height[f] = reference.foot[f].z();
      }
    }
    state = estimator.Update(joints, imu, support, dt);
    model.Measure(
        joints, state,
        0); // FK must correspond to the updated continuous base estimate.
    state.foot_position_world = model.feet();
    if (!state.valid || std::abs(state.euler_zyx.y()) > .35 ||
        std::abs(state.euler_zyx.z()) > .35) {
      fault = true;
      executor.Abort("state_or_attitude");
      Save(now, joints, 0, 0);
      return false;
    }
    if (!tracking) {
      for (size_t k = 0; k < 12; ++k) {
        output.position[k] = initial.position[k];
        output.kp[k] = 60;
        output.kd[k] = 3;
      }
      bool loaded = true;
      for (auto c : contacts)
        loaded = loaded && c.loaded;
      if (now - activated > 1. && loaded) {
        reference.body = state.position;
        reference.euler = state.euler_zyx;
        reference.foot = model.feet();
        support.anchor_position = reference.foot;
        support.anchor_valid.fill(true);
        executor.Reset(reference);
        tracking = true;
      }
      if (now - activated > 5. && !tracking) {
        fault = true;
        executor.Abort("initial_support_unconfirmed");
      }
      Save(now, joints, 0, 0);
      return !fault;
    }
    auto p = *pending.readFromRT();
    if (p.valid && p.step.id != consumed) {
      consumed = p.step.id;
      executor.Start(p.step, now);
    }
    if (cancel.load())
      executor.Abort("canceled");
    reference = executor.Update(now, contacts, model.feet());
    for (size_t f = 0; f < 4; ++f)
      if (reference.contact[f] && contacts[f].loaded &&
          (model.feet()[f] - reference.foot[f]).norm() < .02)
        support.foot_center_height[f] = reference.foot[f].z();
    Eigen::VectorXd q, dq;
    if (!model.Inverse(reference, joints, q, dq)) {
      executor.Abort("runtime_ik");
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
    auto begin = std::chrono::steady_clock::now();
    auto result = wbc->update(desired, input, model.Rbd(joints, state),
                              Mode(reference.contact), dt);
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - begin)
                          .count();
    if (!wbc->lastSolverSucceeded() || wbc->lastEqualityResidual() > 1e-3 ||
        wbc->lastInequalityViolation() > 1e-3 || ms > 20) {
      executor.Abort("wbc_invalid_or_timeout");
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
      executor.Abort("total_torque_limit");
      fault = true;
      Save(now, joints, ms, wbc->lastEqualityResidual());
      return false;
    }
    Save(now, joints, ms, wbc->lastEqualityResidual());
    return true;
  }
  void Save(double now, const JointSample &joints, double ms, double residual) {
    Snapshot s;
    s.now = now;
    s.joints = joints;
    s.estimate = state;
    s.ref = executor.reference();
    s.contact = contacts;
    s.phase = executor.phase();
    s.id = executor.id();
    s.error = executor.error();
    s.solve_ms = ms;
    s.residual = residual;
    s.ready = tracking && !fault &&
              (s.phase == StepPhase::STANCE || s.phase == StepPhase::DONE);
    for (auto c : contacts)
      s.ready = s.ready && c.loaded && !c.slipping;
    snapshot.TryWrite(s);
  }
};
PrecisionRuntime::PrecisionRuntime(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                                   const NmpcBackend &b, const std::string &u,
                                   const std::string &t)
    : impl_(std::make_unique<Impl>(n, b, u, t)) {}
PrecisionRuntime::~PrecisionRuntime() = default;
void PrecisionRuntime::Activate() {
  // Lifecycle transitions run while controller update is inactive. Never
  // resume old trajectories, filters or action goals after reactivation.
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  impl_->initialized = impl_->tracking = impl_->fault = false;
  impl_->busy = impl_->cancel = false;
  impl_->activated = -1;
  impl_->contacts = {};
  impl_->observers = {};
  impl_->support = {};
  impl_->support.foot_center_height.fill(.026);
  impl_->support.height_valid.fill(true);
  impl_->reference = {};
  impl_->state = {};
  impl_->state.position.z() = .29;
  impl_->estimator.Reset(.29);
  impl_->model.Reset();
  impl_->executor = {};
  impl_->approved = {};
  impl_->pending.writeFromNonRT(Pending{});
  impl_->snapshot.Reset();
  impl_->state_pub->on_activate();
  impl_->status_pub->on_activate();
  impl_->contact_pub->on_activate();
  impl_->active = true;
}
void PrecisionRuntime::Deactivate() {
  impl_->active = false;
  std::lock_guard<std::mutex> lock(impl_->callback_mutex);
  if (impl_->goal) {
    auto result = std::make_shared<Action::Result>();
    result->reason = "controller_deactivated";
    if (impl_->goal->is_canceling())
      impl_->goal->canceled(result);
    else if (impl_->goal->is_active())
      impl_->goal->abort(result);
    impl_->goal.reset();
  }
  impl_->busy = false;
  impl_->pending.writeFromNonRT(Pending{});
  impl_->state_pub->on_deactivate();
  impl_->status_pub->on_deactivate();
  impl_->contact_pub->on_deactivate();
}
bool PrecisionRuntime::Update(double n, double d, const JointSample &j,
                              const ImuSample &i, bool stop,
                              HybridJointCommand &c) {
  return impl_->Update(n, d, j, i, stop, c);
}
} // namespace custom_dog_control
