// Non-real-time finite-template client. Every step uses the existing planner
// and action admission path; there is no joint-command publisher here.
#include "qr_planning/FiniteTemplate.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <qr_interfaces/action/execute_footsteps.hpp>
#include <qr_interfaces/msg/execution_status.hpp>
#include <qr_interfaces/msg/robot_state.hpp>
#include <qr_interfaces/srv/plan_footsteps.hpp>
#include <qr_interfaces/srv/preview_footsteps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

using Preview = qr_interfaces::srv::PreviewFootsteps;
using Plan = qr_interfaces::srv::PlanFootsteps;
using Execute = qr_interfaces::action::ExecuteFootsteps;
using Steady = std::chrono::steady_clock;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("qr_finite_sequence");
  const auto file = node->declare_parameter<std::string>("template_file", "");
  const auto result_file =
      node->declare_parameter<std::string>("result_file", "");
  const bool validate_only =
      node->declare_parameter<bool>("validate_only", false);
  const bool coordinated = node->declare_parameter<bool>("coordinated", false);
  const double diagnostic_yaw =
      node->declare_parameter<double>("coordinated_yaw_rad", 0.);
  const double diagnostic_lift =
      node->declare_parameter<double>("coordinated_body_lift_m", 0.);
  YAML::Node report;
  report["coordinated_yaw_rad"] = diagnostic_yaw;
  report["coordinated_body_lift_m"] = diagnostic_lift;
  report["coordinated"] = coordinated;
  report["success"] = false;
  report["completed_steps"] = 0;
  report["hardware_tested"] = false;
  int exit_code = 1;
  try {
    if (!std::isfinite(diagnostic_yaw) || !std::isfinite(diagnostic_lift) ||
        std::abs(diagnostic_yaw) > .1 || diagnostic_lift < 0 ||
        diagnostic_lift > .03)
      throw std::runtime_error("invalid_diagnostic_body_reference");
    if (!result_file.empty() && std::filesystem::exists(result_file))
      throw std::runtime_error("result_file_already_exists");
    const auto steps = qr_planning::ParseFiniteTemplate(YAML::LoadFile(file));
    if (validate_only) {
      rclcpp::shutdown();
      return 0;
    }
    // Precision action server independently rejects real hardware. This client
    // additionally requires simulation time and valid, non-ground-truth state.
    if (!node->get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("simulation_time_required");
    qr_interfaces::msg::RobotState state;
    qr_interfaces::msg::ExecutionStatus status;
    bool got_state = false, got_status = false;
    Steady::time_point state_time{}, status_time{};
    auto state_sub = node->create_subscription<qr_interfaces::msg::RobotState>(
        "/qr/state", 10, [&](qr_interfaces::msg::RobotState::SharedPtr m) {
          state = *m;
          got_state = true;
          state_time = Steady::now();
        });
    auto status_sub =
        node->create_subscription<qr_interfaces::msg::ExecutionStatus>(
            "/qr/execution", 10,
            [&](qr_interfaces::msg::ExecutionStatus::SharedPtr m) {
              status = *m;
              got_status = true;
              status_time = Steady::now();
            });
    auto planner = node->create_client<Plan>("/qr/plan_footsteps");
    auto executor =
        rclcpp_action::create_client<Execute>(node, "/qr/execute_footsteps");
    auto wait = [&](auto predicate, double seconds) {
      const auto deadline =
          Steady::now() + std::chrono::duration<double>(seconds);
      while (rclcpp::ok() && Steady::now() < deadline) {
        rclcpp::spin_some(node);
        if (predicate())
          return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      return false;
    };
    auto ready = [&] {
      const auto now = Steady::now();
      return got_state && got_status && state.valid &&
             !state.using_ground_truth && state.header.frame_id == "odom" &&
             status.ready &&
             now - state_time < std::chrono::milliseconds(200) &&
             now - status_time < std::chrono::milliseconds(200);
    };
    if (!wait(ready, 30.))
      throw std::runtime_error("initial_state_not_ready");
    const auto initial = state.feet;
    auto preview = node->create_client<Preview>("/qr/preview_footsteps");
    if (!preview->wait_for_service(std::chrono::seconds(3)))
      throw std::runtime_error("preview_unavailable");
    auto preview_request = std::make_shared<Preview::Request>();
    for (const auto &step : steps) {
      qr_interfaces::msg::Footstep target;
      target.foot = step.foot;
      target.surface_id = step.surface;
      target.target = initial[step.foot];
      target.target.x += step.offset[0];
      target.target.y += step.offset[1];
      target.target.z += step.offset[2];
      preview_request->steps.push_back(target);
    }
    auto preview_future = preview->async_send_request(preview_request);
    if (!wait(
            [&] {
              return preview_future.wait_for(std::chrono::seconds(0)) ==
                     std::future_status::ready;
            },
            30.))
      throw std::runtime_error("sequence_preview_timeout");
    const auto preview_result = preview_future.get();
    report["preview_accepted"] = preview_result->accepted;
    report["preview_failed_step"] = preview_result->failed_step;
    report["preview_map_revision"] = preview_result->map_revision;
    if (!preview_result->accepted)
      throw std::runtime_error("sequence_preview_rejected: " +
                               preview_result->reason);

    for (size_t index = 0; index < steps.size(); ++index) {
      if (!wait(ready, 3.))
        throw std::runtime_error("support_not_ready_for_next_step");
      const auto &step = steps[index];
      auto request = std::make_shared<Plan::Request>();
      request->foot = step.foot;
      request->surface_id = step.surface;
      request->target = initial[step.foot];
      request->target.x += step.offset[0];
      request->target.y += step.offset[1];
      request->target.z += step.offset[2];
      if (!planner->wait_for_service(std::chrono::seconds(3)))
        throw std::runtime_error("planner_unavailable");
      auto future = planner->async_send_request(request);
      if (!wait(
              [&] {
                return future.wait_for(std::chrono::seconds(0)) ==
                       std::future_status::ready;
              },
              5.))
        throw std::runtime_error("planning_timeout");
      auto planned = future.get();
      if (!planned->accepted)
        throw std::runtime_error("plan_rejected: " + planned->reason);
      if (coordinated) {
        // First plan supplies a validated unload shift. This diagnostic client
        // proposes an explicit body/yaw/foot curve; the server independently
        // validates the complete curve again before granting a new approval.
        const auto &base = planned->plan.steps.front();
        for (size_t k = 0; k < 4; ++k) {
          qr_interfaces::msg::ReferenceKnot knot;
          knot.time_from_start = 2. * k;
          knot.foot = state.feet[step.foot];
          if (k) {
            knot.body.x = base.body_target.x - state.pose.position.x;
            knot.body.y = base.body_target.y - state.pose.position.y;
            knot.body.z = base.body_target.z - state.pose.position.z;
          }
          if (k == 2) {
            knot.foot.x = .5 * (knot.foot.x + request->target.x);
            knot.foot.y = .5 * (knot.foot.y + request->target.y);
            knot.foot.z = std::max(knot.foot.z, request->target.z) + .06;
            knot.body.z += diagnostic_lift;
            knot.euler_zyx.x = diagnostic_yaw;
          }
          if (k == 3)
            knot.foot = request->target;
          request->trajectory.push_back(knot);
        }
        request->lift_index = 1;
        auto coordinated_future = planner->async_send_request(request);
        if (!wait(
                [&] {
                  return coordinated_future.wait_for(std::chrono::seconds(0)) ==
                         std::future_status::ready;
                },
                30.))
          throw std::runtime_error("coordinated_planning_timeout");
        planned = coordinated_future.get();
        if (!planned->accepted)
          throw std::runtime_error("coordinated_plan_rejected: " +
                                   planned->reason);
        report["steps"][index]["trajectory_knots"] =
            planned->plan.steps.front().trajectory.size();
      }
      if (!executor->wait_for_action_server(std::chrono::seconds(3)))
        throw std::runtime_error("executor_unavailable");
      Execute::Goal goal;
      goal.plan = planned->plan;
      auto accepted = executor->async_send_goal(goal);
      if (!wait(
              [&] {
                return accepted.wait_for(std::chrono::seconds(0)) ==
                       std::future_status::ready;
              },
              3.))
        throw std::runtime_error("action_admission_timeout");
      const auto handle = accepted.get();
      if (!handle)
        throw std::runtime_error("action_rejected");
      auto result = executor->async_get_result(handle);
      if (!wait(
              [&] {
                return result.wait_for(std::chrono::seconds(0)) ==
                       std::future_status::ready;
              },
              30.)) {
        // Cancellation follows the same controlled-abort action contract. No
        // next step is issued even if a disconnected server cannot acknowledge.
        auto cancel = executor->async_cancel_goal(handle);
        wait(
            [&] {
              return cancel.wait_for(std::chrono::seconds(0)) ==
                     std::future_status::ready;
            },
            2.);
        throw std::runtime_error("action_timeout_cancel_requested");
      }
      const auto completed = result.get();
      if (!completed.result)
        throw std::runtime_error("missing_action_result");
      report["steps"][index]["foot"] = static_cast<unsigned>(step.foot);
      report["steps"][index]["plan_id"] = goal.plan.id;
      report["steps"][index]["surface_id"] = step.surface;
      for (const auto &p : goal.plan.steps.front().safe_region.points)
        report["steps"][index]["safe_region"].push_back(
            std::vector<double>{p.x, p.y, p.z});
      report["steps"][index]["target"] = std::vector<double>{
          request->target.x, request->target.y, request->target.z};
      report["steps"][index]["reason"] = completed.result->reason;
      if (completed.code != rclcpp_action::ResultCode::SUCCEEDED ||
          !completed.result->success)
        throw std::runtime_error("execution_aborted: " +
                                 completed.result->reason);
      if (!wait([&] { return ready() && status.plan_id == goal.plan.id; }, 3.))
        throw std::runtime_error("completed_step_support_not_confirmed");
      report["completed_steps"] = index + 1;
      RCLCPP_INFO(node->get_logger(), "Completed %zu/%zu: foot %u, plan %lu",
                  index + 1, steps.size(), step.foot, goal.plan.id);
    }
    report["success"] = true;
    exit_code = 0;
  } catch (const std::exception &e) {
    report["error"] = e.what();
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
  }
  if (!result_file.empty() && !std::filesystem::exists(result_file)) {
    std::ofstream out(result_file);
    out << report;
    if (!out)
      exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
