#include "custom_dog_control/controller/NmpcWbcController.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace custom_dog_control {
namespace {

double LimitAcceleratingReference(
    double requested, double measured, double max_lead) {
  if (requested > 0.0) {
    return std::min(requested, std::max(0.0, measured) + max_lead);
  }
  if (requested < 0.0) {
    return std::max(requested, std::min(0.0, measured) - max_lead);
  }
  return 0.0;
}

}  // namespace

controller_interface::return_type NmpcWbcController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  const double now_seconds = time.seconds();
  const double dt = std::clamp(period.seconds(), 1e-4, 0.02);
  control_period_ms_ = period.seconds() * 1000.0;
  control_timing_.Add(control_period_ms_);
  ReadHardwareState();
  if (IsRealHardware()) {
    io_timing_.Add(io_period_ms_);
  }
  imu_sample_ = *imu_buffer_.readFromRT();
  const JoyInput joy = *joy_buffer_.readFromRT();
  const VelocityCommand command = SelectVelocityCommand(now_seconds, dt);

  std::size_t planned_mode = last_policy_.valid ? last_policy_.mode : kStanceMode;
  const auto contacts = ContactFlags(planned_mode);
  if (!IsRealHardware() && use_sim_ground_truth_) {
    estimate_ = *ground_truth_buffer_.readFromRT();
    estimate_.valid =
        estimate_.valid &&
        now_seconds - estimate_.stamp_seconds <= ground_truth_timeout_s_;
  } else {
    estimate_ = estimator_->Update(joints_state_, imu_sample_, contacts, dt);
  }
  if (estimate_.valid) {
    observation_time_ += dt;
    measured_rbd_state_ = backend_->UpdateObservation(
        estimate_, joints_state_, observation_time_, planned_mode);
    // The operator command must request the gait transition immediately, but
    // the moving world-frame target must not run ahead while NMPC is still
    // producing a stance policy. Start integrating it only after the first
    // Trot policy has been accepted by the locomotion supervisor.
    VelocityCommand backend_command = command;
    if (mode_ != OperatingMode::MPC_TROT) {
      backend_command.vx = 0.0;
      backend_command.vy = 0.0;
      backend_command.yaw = 0.0;
    } else {
      const double yaw = estimate_.euler_zyx.x();
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double measured_vx_body =
          cos_yaw * estimate_.velocity_world.x() +
          sin_yaw * estimate_.velocity_world.y();
      const double measured_vy_body =
          -sin_yaw * estimate_.velocity_world.x() +
          cos_yaw * estimate_.velocity_world.y();
      backend_command.vx = LimitAcceleratingReference(
          command.vx, measured_vx_body, max_reference_lead_xy_m_s_);
      backend_command.vy = LimitAcceleratingReference(
          command.vy, measured_vy_body, max_reference_lead_xy_m_s_);
      backend_command.yaw = LimitAcceleratingReference(
          command.yaw, estimate_.angular_velocity_world.z(),
          max_reference_lead_yaw_rad_s_);
    }
    governed_command_ = backend_command;
    const bool target_command_is_moving =
        std::max({std::abs(backend_command.vx),
                  std::abs(backend_command.vy),
                  std::abs(backend_command.yaw)}) > 1e-3;
    const bool moving_target_update_due =
        target_command_is_moving &&
        now_seconds - last_target_update_seconds_ >= 0.05;
    const double stance_anchor_error = std::hypot(
        estimate_.position.x() - stance_anchor_xy_[0],
        estimate_.position.y() - stance_anchor_xy_[1]);
    const bool stance_reanchor_due =
        mode_ == OperatingMode::MPC_STANCE && !target_command_is_moving &&
        stance_anchor_error >= stance_reanchor_distance_m_;
    // Keep a stable reference between updates. Re-anchoring is event driven
    // below so the 50 Hz solver is not continuously restarted while WBC is
    // taking over from the position-controlled stand.
    if (last_target_update_seconds_ < 0.0 || moving_target_update_due ||
        stance_reanchor_due ||
        (target_command_was_moving_ && !target_command_is_moving)) {
      backend_->SetVelocityCommand(
          backend_command,
          stance_reanchor_due ||
              (target_command_was_moving_ && !target_command_is_moving));
      last_target_update_seconds_ = now_seconds;
      if (!target_command_is_moving) {
        stance_anchor_xy_ = {estimate_.position.x(), estimate_.position.y()};
      }
    }
    target_command_was_moving_ = target_command_is_moving;
  }

  PolicySample evaluated_policy;
  const bool evaluated = estimate_.valid &&
                         backend_->EvaluatePolicy(now_seconds, evaluated_policy);
  if (evaluated) {
    policy_buffer_.writeFromNonRT(evaluated_policy);
    if (evaluated_policy.sequence != last_timed_policy_sequence_) {
      mpc_timing_.Add(evaluated_policy.solve_time_ms);
      last_timed_policy_sequence_ = evaluated_policy.sequence;
    }
  }
  last_policy_ = *policy_buffer_.readFromRT();
  const bool policy_available = last_policy_.valid;
  const double policy_age = backend_->policyAgeSeconds(now_seconds);

  if (mode_ != OperatingMode::PASSIVE &&
      mode_ != OperatingMode::CALIBRATION &&
      mode_ != OperatingMode::FAULT) {
    SafetyInput safety_input;
    safety_input.joints = joints_state_;
    safety_input.imu = imu_sample_;
    safety_input.now_seconds = now_seconds;
    safety_input.policy_age_s = policy_age;
    safety_input.roll = estimate_.euler_zyx.z();
    safety_input.pitch = estimate_.euler_zyx.y();
    safety_input.consecutive_io_failures = consecutive_io_failures_;
    safety_input.communication_ok = communication_ok_;
    safety_input.physical_estop = physical_estop_;
    safety_input.solver_valid = backend_->solverHealthy();
    safety_input.dynamic_mode = mode_ == OperatingMode::MPC_TROT;
    if (!safety_monitor_->Evaluate(safety_input)) {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Safety fault: %s",
          safety_monitor_->reason().c_str());
      TransitionTo(OperatingMode::FAULT, now_seconds);
    }
  }

  if (mode_ != OperatingMode::FAULT) {
    UpdateLocomotionSupervisor(
        now_seconds, command, joy, last_policy_, policy_age);
  }

  HybridJointCommand hybrid_command;
  WbcOutput wbc;
  const bool state_ok = ApplyStateMachine(
      now_seconds, dt, joy, last_policy_, policy_available,
      hybrid_command, wbc);
  if (!state_ok || mode_ == OperatingMode::PASSIVE ||
      mode_ == OperatingMode::CALIBRATION ||
      mode_ == OperatingMode::FAULT) {
    WriteSafeCommand();
  } else {
    WriteHybridCommand(hybrid_command);
  }

  PublishDiagnostics(time, estimate_, last_policy_, wbc);
  return controller_interface::return_type::OK;
}

}  // namespace custom_dog_control

PLUGINLIB_EXPORT_CLASS(
    custom_dog_control::NmpcWbcController,
    controller_interface::ControllerInterface)
