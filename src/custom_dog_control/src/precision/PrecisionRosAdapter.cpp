#include "custom_dog_control/precision/PrecisionRosAdapter.hpp"
#include <mutex>
#include <qr_interfaces/action/execute_footsteps.hpp>
#include <qr_interfaces/msg/foot_contact_array.hpp>
#include <qr_interfaces/msg/robot_state.hpp>
#include <qr_interfaces/msg/support_region_array.hpp>
#include <qr_interfaces/srv/plan_footsteps.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
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
} // namespace
struct PrecisionRosAdapter::Impl {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  PrecisionPlanner &preflight;
  PrecisionChannel &channel;
  PrecisionConfig config;
  std::mutex callback_mutex;
  qr_interfaces::msg::SupportRegionArray regions;
  qr_interfaces::msg::FootstepPlan approved;
  PrecisionStep approved_step;
  uint64_t next_id = 1, accepted_id = 0;
  bool busy = false;
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
  Impl(rclcpp_lifecycle::LifecycleNode::SharedPtr n, PrecisionPlanner &p,
       PrecisionChannel &c, const PrecisionConfig &settings)
      : node(n), preflight(p), channel(c), config(settings) {
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
          auto s = channel.snapshot.Read();
          auto reject = [&](const char *reason) {
            RCLCPP_WARN(
                node->get_logger(),
                "QR_ADMISSION_REJECT plan=%lu reason=%s snapshot_age_s=%.6f",
                request->plan.id, reason,
                PrecisionSteadyNow() - s.steady_stamp);
            return rclcpp_action::GoalResponse::REJECT;
          };
          if (!channel.active)
            return reject("inactive");
          if (busy)
            return reject("busy");
          if (!s.ready)
            return reject("state_not_ready");
          if (request->plan != approved)
            return reject("plan_not_approved");
          if (request->plan.id == accepted_id)
            return reject("replay");
          if (regions.revision != approved.map_revision)
            return reject("map_revision_changed");
          if (!SnapshotFresh(s, PrecisionSteadyNow(), config.state_timeout_s))
            return reject("stale_snapshot");
          if (approved.steps.size() != 1)
            return reject("unsupported_step_count");
          if (node->now() > rclcpp::Time(approved.valid_until))
            return reject("expired_plan");
          if ((s.ref.body - approved_reference.body).norm() >
              config.body_drift_m)
            return reject("body_reference_drift");
          for (size_t f = 0; f < 4; ++f)
            if ((s.estimate.foot_position_world[f] - approved_reference.foot[f])
                    .norm() > config.target_tolerance_m)
              return reject("foot_state_drift");
          accepted_id = approved.id;
          channel.cancel = false;
          busy = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<Goal>) {
          channel.cancel = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<Goal> g) {
          std::lock_guard<std::mutex> lock(callback_mutex);
          goal = g;
          channel.pending.writeFromNonRT(PrecisionCommand{approved_step, true});
        });
    timer = node->create_wall_timer(
        std::chrono::duration<double>(config.status_period_s),
        [this] { Publish(); });
  }
  void Plan(const qr_interfaces::srv::PlanFootsteps::Request &req,
            qr_interfaces::srv::PlanFootsteps::Response &res) {
    const auto s = channel.snapshot.Read();
    const double now = node->now().seconds();
    if (!channel.active || busy || !s.ready ||
        !SnapshotFresh(s, PrecisionSteadyNow(), config.state_timeout_s)) {
      res.error_code = static_cast<uint16_t>(PrecisionError::STATE_NOT_READY);
      res.reason = ErrorName(PrecisionError::STATE_NOT_READY);
      return;
    }
    if (req.foot >= 4 || regions.header.frame_id != "odom") {
      res.error_code =
          static_cast<uint16_t>(PrecisionError::INVALID_FOOT_OR_FRAME);
      res.reason = ErrorName(PrecisionError::INVALID_FOOT_OR_FRAME);
      return;
    }
    std::vector<PlanningSurface> surfaces;
    for (const auto &source : regions.regions) {
      PlanningSurface surface;
      surface.id = source.id;
      surface.known = source.known;
      surface.forbidden = source.forbidden;
      surface.confidence = source.confidence;
      surface.observed_at = rclcpp::Time(source.observed_at).seconds();
      surface.normal = {source.normal.x, source.normal.y, source.normal.z};
      for (const auto &p : source.polygon.points)
        surface.points.push_back({p.x, p.y, p.z});
      surfaces.push_back(std::move(surface));
    }
    const auto result = preflight.Plan(
        {req.foot, req.surface_id, Vec(req.target)}, s, surfaces, now);
    res.accepted = result.accepted;
    res.error_code = static_cast<uint16_t>(result.error);
    res.reason =
        result.accepted ? "validated_single_step" : ErrorName(result.error);
    if (!result.accepted)
      return;
    approved_step = result.step;
    approved_step.id = next_id++;
    qr_interfaces::msg::Footstep out;
    out.foot = req.foot;
    out.surface_id = req.surface_id;
    out.target = req.target;
    for (const auto &p : result.safe_region) {
      geometry_msgs::msg::Point32 point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      out.safe_region.points.push_back(point);
    }
    out.body_target = Point(approved_step.body);
    out.shift_duration = approved_step.shift;
    out.swing_duration = approved_step.swing;
    out.clearance = approved_step.clearance;
    out.contact_timeout = approved_step.timeout;
    approved = qr_interfaces::msg::FootstepPlan{};
    approved.header.frame_id = "odom";
    approved.header.stamp = node->now();
    approved.id = approved_step.id;
    approved.map_revision = regions.revision;
    approved.valid_until =
        node->now() + rclcpp::Duration::from_seconds(config.plan_validity_s);
    approved.steps = {out};
    approved_reference = s.ref;
    res.plan = approved;
  }
  void Publish() {
    std::lock_guard<std::mutex> lock(callback_mutex);
    if (!channel.active)
      return;
    auto s = channel.snapshot.Read();
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
    status.protocol_version = 1;
    status.phase_code = static_cast<uint8_t>(s.phase);
    status.error_code = static_cast<uint16_t>(s.error);
    status.error = ErrorName(s.error);
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
    if (goal && s.id == approved.id) {
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->status = status;
      goal->publish_feedback(feedback);
      if (s.phase == StepPhase::DONE || s.phase == StepPhase::HOLD) {
        auto result = std::make_shared<Action::Result>();
        result->success = s.phase == StepPhase::DONE;
        result->reason = ErrorName(s.error);
        result->error_code = static_cast<uint16_t>(s.error);
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
  void Activate() {
    std::lock_guard<std::mutex> lock(callback_mutex);
    busy = false;
    channel.cancel = false;
    approved = qr_interfaces::msg::FootstepPlan{};
    channel.pending.writeFromNonRT(PrecisionCommand{});
    channel.snapshot.Reset();
    state_pub->on_activate();
    status_pub->on_activate();
    contact_pub->on_activate();
    channel.active = true;
  }
  void Deactivate() {
    channel.active = false;
    std::lock_guard<std::mutex> lock(callback_mutex);
    if (goal) {
      auto result = std::make_shared<Action::Result>();
      result->reason = ErrorName(PrecisionError::CONTROLLER_DEACTIVATED);
      result->error_code =
          static_cast<uint16_t>(PrecisionError::CONTROLLER_DEACTIVATED);
      if (goal->is_canceling())
        goal->canceled(result);
      else if (goal->is_active())
        goal->abort(result);
      goal.reset();
    }
    busy = false;
    channel.pending.writeFromNonRT(PrecisionCommand{});
    state_pub->on_deactivate();
    status_pub->on_deactivate();
    contact_pub->on_deactivate();
  }
};
PrecisionRosAdapter::PrecisionRosAdapter(
    rclcpp_lifecycle::LifecycleNode::SharedPtr n, PrecisionPlanner &p,
    PrecisionChannel &c, const PrecisionConfig &settings)
    : impl_(std::make_unique<Impl>(n, p, c, settings)) {}
PrecisionRosAdapter::~PrecisionRosAdapter() = default;
void PrecisionRosAdapter::Activate() { impl_->Activate(); }
void PrecisionRosAdapter::Deactivate() { impl_->Deactivate(); }
} // namespace custom_dog_control
