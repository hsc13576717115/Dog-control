#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/control/TimingStatistics.hpp"
#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"
#include "custom_dog_control/nmpc/NmpcBackend.hpp"
#include "custom_dog_control/safety/SafetyMonitor.hpp"

namespace custom_dog_control {

class NmpcWbcController final : public controller_interface::ControllerInterface {
 public:
  ~NmpcWbcController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  enum class RequestedMode : std::uint8_t {
    NONE,
    PASSIVE,
    CALIBRATE_AND_STAND,
    STANCE,
    TROT,
    ESTOP,
  };

  struct JoyInput {
    VelocityCommand velocity;
    RequestedMode requested_mode = RequestedMode::NONE;
    double stamp_seconds = 0.0;
    bool active = false;
  };

  enum class GaitTransition : std::uint8_t {
    NONE,
    STARTING_TROT,
    STOPPING_TROT,
  };

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr message);
  void GroundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr message);
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr message);
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr message);
  void TransitionTo(OperatingMode next, double now_seconds);
  bool ResolveInterfaceIndices();
  void ReadHardwareState();
  VelocityCommand SelectVelocityCommand(double now_seconds, double dt);
  void UpdateLocomotionSupervisor(
      double now_seconds, const VelocityCommand& command, const JoyInput& joy,
      const PolicySample& policy, double policy_age_seconds);
  void WriteHybridCommand(const HybridJointCommand& command);
  void WriteSafeCommand(bool reset_calibration = false);
  bool ApplyStateMachine(
      double now_seconds, double dt, const JoyInput& joy,
      const PolicySample& policy, bool policy_available,
      HybridJointCommand& command, WbcOutput& wbc);
  void PublishDiagnostics(
      const rclcpp::Time& stamp, const EstimatedState& estimate,
      const PolicySample& policy, const WbcOutput& wbc);
  bool IsRealHardware() const { return hardware_mode_ == "real"; }

  std::vector<std::string> joints_;
  std::string hardware_mode_ = "gazebo";
  std::string task_file_;
  std::string reference_file_;
  std::string urdf_file_;
  std::string generated_urdf_file_;
  bool legacy_joy_y_right_ = true;
  bool use_sim_ground_truth_ = true;
  double ground_truth_timeout_s_ = 0.10;

  VelocityLimits velocity_limits_;
  double command_timeout_s_ = 0.30;
  double joy_timeout_s_ = 0.30;
  double max_reference_lead_xy_m_s_ = 0.25;
  double max_reference_lead_yaw_rad_s_ = 0.40;
  double stand_up_duration_s_ = 2.5;
  double stand_up_settle_duration_s_ = 1.0;
  double stand_up_hip_kp_ = 60.0;
  double stand_up_leg_kp_ = 60.0;
  double stand_up_calf_kp_ = 30.0;
  double stand_up_kd_ = 3.0;
  double stand_up_settle_hip_kp_ = 80.0;
  double stand_up_settle_leg_kp_ = 80.0;
  double stand_up_settle_calf_kp_ = 80.0;
  double stand_up_settle_kd_ = 5.0;
  double stand_up_position_tolerance_rad_ = 0.55;
  double stand_up_velocity_tolerance_rad_s_ = 0.20;
  double stand_up_base_velocity_tolerance_m_s_ = 0.05;
  double handoff_duration_s_ = 1.0;
  bool stance_handoff_active_ = false;
  double wbc_joint_stiffness_ = 0.0;
  double wbc_hip_stiffness_ = 0.0;
  double wbc_joint_damping_ = 3.0;
  double stance_reanchor_distance_m_ = 1.0;
  double trot_entry_dwell_s_ = 0.15;
  double trot_exit_dwell_s_ = 0.30;
  int max_consecutive_wbc_failures_ = 5;
  int consecutive_wbc_failures_ = 0;
  double nominal_height_m_ = 0.28;
  double safe_damping_ = 1.0;
  bool simulation_passive_hold_ = true;
  double simulation_passive_hip_kp_ = 20.0;
  double simulation_passive_leg_kp_ = 60.0;
  double simulation_passive_kd_ = 3.0;
  double state_entered_seconds_ = 0.0;
  double observation_time_ = 0.0;
  double last_target_update_seconds_ = -1.0;
  bool target_command_was_moving_ = false;
  std::array<double, 2> stance_anchor_xy_{};
  double last_diagnostics_seconds_ = -1.0;
  std::array<double, kJointCount> nominal_joint_positions_{};
  std::array<double, kJointCount> stand_up_joint_positions_{};
  std::array<double, kJointCount> passive_joint_positions_{};
  std::array<double, kJointCount> stand_start_positions_{};
  std::array<std::array<std::size_t, 5>, kJointCount> command_interface_indices_{};
  std::array<std::array<std::size_t, 5>, kJointCount> state_interface_indices_{};
  std::array<std::size_t, 2> system_command_interface_indices_{};
  std::array<std::size_t, 5> system_state_interface_indices_{};

  OperatingMode mode_ = OperatingMode::PASSIVE;
  bool requires_recalibration_ = true;
  bool reset_calibration_pending_ = false;
  bool calibrated_ = false;
  bool communication_ok_ = true;
  bool physical_estop_ = false;
  int consecutive_io_failures_ = 0;
  double io_period_ms_ = 0.0;
  double io_timeout_count_ = 0.0;
  double control_period_ms_ = 0.0;
  std::uint64_t last_timed_policy_sequence_ = 0;
  std::uint64_t gait_request_policy_sequence_ = 0;
  bool trot_enabled_ = false;
  GaitTransition gait_transition_ = GaitTransition::NONE;
  double gait_condition_since_seconds_ = -1.0;
  VelocityCommand limited_command_;
  VelocityCommand governed_command_;
  JointSample joints_state_;
  ImuSample imu_sample_;
  PolicySample last_policy_;
  ocs2::vector_t measured_rbd_state_;
  EstimatedState estimate_;
  TimingWindow<512> control_timing_;
  TimingWindow<512> io_timing_;
  TimingWindow<512> mpc_timing_;

  std::unique_ptr<NmpcBackend> backend_;
  std::unique_ptr<KinematicStateEstimator> estimator_;
  std::unique_ptr<SafetyMonitor> safety_monitor_;

  realtime_tools::RealtimeBuffer<ImuSample> imu_buffer_;
  realtime_tools::RealtimeBuffer<EstimatedState> ground_truth_buffer_;
  realtime_tools::RealtimeBuffer<JoyInput> joy_buffer_;
  realtime_tools::RealtimeBuffer<VelocityCommand> cmd_vel_buffer_;
  realtime_tools::RealtimeBuffer<PolicySample> policy_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      ground_truth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      contact_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace custom_dog_control
