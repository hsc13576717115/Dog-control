#include "custom_dog_control/controller/NmpcWbcController.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "custom_dog_control/hardware/Calibration.hpp"

namespace custom_dog_control {
namespace {

std::vector<std::string> DefaultJointNames() {
  std::vector<std::string> result;
  result.reserve(kJointCount);
  for (const auto name : kJointNames) {
    result.emplace_back(name);
  }
  return result;
}

std::array<double, kJointCount> DefaultNominalJointPositions() {
  return {
       0.0, 0.873118638, -1.776967680,
       0.0, 0.872802956, -1.777230490,
       0.0, 0.872609587, -1.777385860,
       0.0, 0.870485702, -1.779097810};
}

std::array<double, kJointCount> DefaultStandUpJointPositions() {
  return {
       0.026376720, 0.876889620, -1.709903390,
      -0.026442310, 0.876586360, -1.710184030,
       0.026459680, 0.876378750, -1.710350660,
      -0.026379650, 0.874245170, -1.712185710};
}

std::array<double, kJointCount> DefaultPassiveJointPositions() {
  std::array<double, kJointCount> result{};
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    std::copy(
        kProneCalibrationPose.begin(), kProneCalibrationPose.end(),
        result.begin() + leg * kJointsPerLeg);
  }
  return result;
}

bool Button(const sensor_msgs::msg::Joy& message, std::size_t index) {
  return index < message.buttons.size() && message.buttons[index] != 0;
}

double Axis(const sensor_msgs::msg::Joy& message, std::size_t index) {
  return index < message.axes.size() ? message.axes[index] : 0.0;
}

double SmoothStep(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

diagnostic_msgs::msg::KeyValue KeyValue(
    const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

std::string Number(double value) {
  std::ostringstream stream;
  stream.precision(6);
  stream << value;
  return stream.str();
}

Eigen::Quaterniond QuaternionFromZyx(const Eigen::Vector3d& zyx) {
  return Eigen::AngleAxisd(zyx.x(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(zyx.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(zyx.z(), Eigen::Vector3d::UnitX());
}

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

NmpcWbcController::~NmpcWbcController() {
  if (backend_) {
    backend_->Stop();
  }
  if (!generated_urdf_file_.empty()) {
    std::remove(generated_urdf_file_.c_str());
  }
}

controller_interface::CallbackReturn NmpcWbcController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", DefaultJointNames());
    auto_declare<std::string>("hardware_mode", "gazebo");
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("urdf_file", "");
    auto_declare<std::string>("task_file", "");
    auto_declare<std::string>("reference_file", "");
    auto_declare<bool>("legacy_joy_y_right", true);
    auto_declare<bool>("use_sim_ground_truth", true);
    auto_declare<double>("ground_truth_timeout_s", 0.10);
    // legged_control task.info uses mpcDesiredFrequency=50 Hz. The WBC is
    // evaluated from update() at the controller_manager rate.
    auto_declare<double>("mpc_frequency_hz", 50.0);
    auto_declare<double>("simulation_policy_timeout_cycles", 4.0);
    auto_declare<double>("command_timeout_s", 0.30);
    auto_declare<double>("joy_timeout_s", 0.30);
    auto_declare<double>("max_reference_lead_xy_m_s", 0.25);
    auto_declare<double>("max_reference_lead_yaw_rad_s", 0.40);
    auto_declare<double>("stand_up_duration_s", 2.50);
    auto_declare<double>("stand_up_settle_duration_s", 2.0);
    auto_declare<double>("stand_up_hip_kp", 45.0);
    auto_declare<double>("stand_up_leg_kp", 45.0);
    auto_declare<double>("stand_up_calf_kp", 45.0);
    auto_declare<double>("stand_up_kd", 1.5);
    auto_declare<double>("stand_up_settle_hip_kp", 55.0);
    auto_declare<double>("stand_up_settle_leg_kp", 55.0);
    auto_declare<double>("stand_up_settle_calf_kp", 55.0);
    auto_declare<double>("stand_up_settle_kd", 2.5);
    auto_declare<double>("stand_up_position_tolerance_rad", 0.55);
    auto_declare<double>("stand_up_velocity_tolerance_rad_s", 0.20);
    auto_declare<double>("stand_up_base_velocity_tolerance_m_s", 0.05);
    auto_declare<double>("handoff_duration_s", 1.0);
    auto_declare<double>("wbc_joint_stiffness", 0.0);
    auto_declare<double>("wbc_hip_stiffness", 0.0);
    auto_declare<double>("wbc_joint_damping", 3.0);
    auto_declare<double>("locomotion_max_hip_angle_rad", 0.20);
    auto_declare<double>("locomotion_joint_limit_margin_rad", 0.03);
    auto_declare<double>("stance_reanchor_distance_m", 1.0);
    auto_declare<double>("trot_entry_dwell_s", 0.15);
    auto_declare<double>("trot_exit_dwell_s", 0.30);
    auto_declare<int>("max_consecutive_wbc_failures", 5);
    auto_declare<double>("nominal_height_m", 0.28);
    auto_declare<double>("safe_damping", 1.0);
    auto_declare<bool>("simulation_passive_hold", true);
    auto_declare<double>("simulation_passive_hip_kp", 20.0);
    auto_declare<double>("simulation_passive_leg_kp", 60.0);
    auto_declare<double>("simulation_passive_kd", 3.0);
    auto_declare<double>("velocity_limit_x", 2.0);
    auto_declare<double>("velocity_limit_y", 1.2);
    auto_declare<double>("velocity_limit_yaw", 2.0);
    auto_declare<double>("acceleration_limit_xy", 1.0);
    auto_declare<double>("acceleration_limit_yaw", 1.5);
    auto_declare<double>("joint_limit_tolerance_rad", 0.005);
    const auto defaults = DefaultNominalJointPositions();
    auto_declare<std::vector<double>>(
        "nominal_joint_positions",
        std::vector<double>(defaults.begin(), defaults.end()));
    const auto stand_defaults = DefaultStandUpJointPositions();
    auto_declare<std::vector<double>>(
        "stand_up_joint_positions",
        std::vector<double>(stand_defaults.begin(), stand_defaults.end()));
    const auto passive_defaults = DefaultPassiveJointPositions();
    auto_declare<std::vector<double>>(
        "passive_joint_positions",
        std::vector<double>(passive_defaults.begin(), passive_defaults.end()));
    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

controller_interface::InterfaceConfiguration
NmpcWbcController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  if (IsRealHardware()) {
    configuration.names.reserve(kJointCount * 5 + 2);
    for (const auto& joint : joints_) {
      configuration.names.push_back(joint + "/position");
      configuration.names.push_back(joint + "/velocity");
      configuration.names.push_back(joint + "/effort");
      configuration.names.push_back(joint + "/kp");
      configuration.names.push_back(joint + "/kd");
    }
    configuration.names.push_back("custom_dog/calibrate");
    configuration.names.push_back("custom_dog/emergency_stop");
  } else {
    configuration.names.reserve(kJointCount);
    for (const auto& joint : joints_) {
      configuration.names.push_back(joint + "/effort");
    }
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
NmpcWbcController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  if (IsRealHardware()) {
    configuration.names.reserve(kJointCount * 5 + 5);
    for (const auto& joint : joints_) {
      configuration.names.push_back(joint + "/position");
      configuration.names.push_back(joint + "/velocity");
      configuration.names.push_back(joint + "/effort");
      configuration.names.push_back(joint + "/temperature");
      configuration.names.push_back(joint + "/valid");
    }
    configuration.names.push_back("custom_dog/calibrated");
    configuration.names.push_back("custom_dog/communication_ok");
    configuration.names.push_back("custom_dog/physical_estop");
    configuration.names.push_back("custom_dog/io_period_ms");
    configuration.names.push_back("custom_dog/io_timeout_count");
  } else {
    configuration.names.reserve(kJointCount * 3);
    for (const auto& joint : joints_) {
      configuration.names.push_back(joint + "/position");
      configuration.names.push_back(joint + "/velocity");
      configuration.names.push_back(joint + "/effort");
    }
  }
  return configuration;
}

controller_interface::CallbackReturn NmpcWbcController::on_configure(
    const rclcpp_lifecycle::State&) {
  try {
    const auto node = get_node();
    joints_ = node->get_parameter("joints").as_string_array();
    hardware_mode_ = node->get_parameter("hardware_mode").as_string();
    if (joints_.size() != kJointCount ||
        (hardware_mode_ != "real" && hardware_mode_ != "gazebo")) {
      throw std::invalid_argument("joints must contain 12 entries and hardware_mode must be real or gazebo");
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (joints_[i] != kJointNames[i]) {
        throw std::invalid_argument("joint order must be FR, FL, RR, RL with hip, thigh, calf");
      }
    }

    legacy_joy_y_right_ = node->get_parameter("legacy_joy_y_right").as_bool();
    use_sim_ground_truth_ =
        node->get_parameter("use_sim_ground_truth").as_bool();
    ground_truth_timeout_s_ =
        node->get_parameter("ground_truth_timeout_s").as_double();
    command_timeout_s_ = node->get_parameter("command_timeout_s").as_double();
    joy_timeout_s_ = node->get_parameter("joy_timeout_s").as_double();
    max_reference_lead_xy_m_s_ =
        node->get_parameter("max_reference_lead_xy_m_s").as_double();
    max_reference_lead_yaw_rad_s_ =
        node->get_parameter("max_reference_lead_yaw_rad_s").as_double();
    stand_up_duration_s_ = node->get_parameter("stand_up_duration_s").as_double();
    stand_up_settle_duration_s_ =
        node->get_parameter("stand_up_settle_duration_s").as_double();
    stand_up_hip_kp_ = node->get_parameter("stand_up_hip_kp").as_double();
    stand_up_leg_kp_ = node->get_parameter("stand_up_leg_kp").as_double();
    stand_up_calf_kp_ = node->get_parameter("stand_up_calf_kp").as_double();
    stand_up_kd_ = node->get_parameter("stand_up_kd").as_double();
    stand_up_settle_hip_kp_ =
        node->get_parameter("stand_up_settle_hip_kp").as_double();
    stand_up_settle_leg_kp_ =
        node->get_parameter("stand_up_settle_leg_kp").as_double();
    stand_up_settle_calf_kp_ =
        node->get_parameter("stand_up_settle_calf_kp").as_double();
    stand_up_settle_kd_ =
        node->get_parameter("stand_up_settle_kd").as_double();
    stand_up_position_tolerance_rad_ =
        node->get_parameter("stand_up_position_tolerance_rad").as_double();
    stand_up_velocity_tolerance_rad_s_ =
        node->get_parameter("stand_up_velocity_tolerance_rad_s").as_double();
    stand_up_base_velocity_tolerance_m_s_ = node->get_parameter(
        "stand_up_base_velocity_tolerance_m_s").as_double();
    handoff_duration_s_ = node->get_parameter("handoff_duration_s").as_double();
    wbc_joint_stiffness_ =
        node->get_parameter("wbc_joint_stiffness").as_double();
    wbc_hip_stiffness_ =
        node->get_parameter("wbc_hip_stiffness").as_double();
    wbc_joint_damping_ = node->get_parameter("wbc_joint_damping").as_double();
    stance_reanchor_distance_m_ =
        node->get_parameter("stance_reanchor_distance_m").as_double();
    trot_entry_dwell_s_ =
        node->get_parameter("trot_entry_dwell_s").as_double();
    trot_exit_dwell_s_ =
        node->get_parameter("trot_exit_dwell_s").as_double();
    max_consecutive_wbc_failures_ =
        static_cast<int>(node->get_parameter("max_consecutive_wbc_failures").as_int());
    nominal_height_m_ = node->get_parameter("nominal_height_m").as_double();
    safe_damping_ = node->get_parameter("safe_damping").as_double();
    simulation_passive_hold_ =
        node->get_parameter("simulation_passive_hold").as_bool();
    simulation_passive_hip_kp_ =
        node->get_parameter("simulation_passive_hip_kp").as_double();
    simulation_passive_leg_kp_ =
        node->get_parameter("simulation_passive_leg_kp").as_double();
    simulation_passive_kd_ =
        node->get_parameter("simulation_passive_kd").as_double();
    if (ground_truth_timeout_s_ <= 0.0 ||
        max_reference_lead_xy_m_s_ <= 0.0 ||
        max_reference_lead_yaw_rad_s_ <= 0.0 ||
        stand_up_duration_s_ <= 0.0 || stand_up_settle_duration_s_ < 0.0 ||
        stand_up_hip_kp_ < 0.0 || stand_up_leg_kp_ < 0.0 ||
        stand_up_calf_kp_ < 0.0 ||
        stand_up_kd_ < 0.0 || stand_up_settle_hip_kp_ < 0.0 ||
        stand_up_settle_leg_kp_ < 0.0 ||
        stand_up_settle_calf_kp_ < 0.0 || stand_up_settle_kd_ < 0.0 ||
        stand_up_position_tolerance_rad_ <= 0.0 ||
        stand_up_velocity_tolerance_rad_s_ <= 0.0 ||
        stand_up_base_velocity_tolerance_m_s_ <= 0.0 ||
        wbc_joint_stiffness_ < 0.0 || wbc_hip_stiffness_ < 0.0 ||
        wbc_joint_damping_ < 0.0 ||
        stance_reanchor_distance_m_ <= 0.0 ||
        trot_entry_dwell_s_ < 0.0 || trot_exit_dwell_s_ < 0.0 ||
        max_consecutive_wbc_failures_ <= 0 ||
        safe_damping_ < 0.0 ||
        simulation_passive_hip_kp_ < 0.0 ||
        simulation_passive_leg_kp_ < 0.0 || simulation_passive_kd_ < 0.0) {
      throw std::invalid_argument("stand-up timing and position gains are invalid");
    }
    velocity_limits_.vx = node->get_parameter("velocity_limit_x").as_double();
    velocity_limits_.vy = node->get_parameter("velocity_limit_y").as_double();
    velocity_limits_.yaw = node->get_parameter("velocity_limit_yaw").as_double();
    velocity_limits_.acceleration_xy =
        node->get_parameter("acceleration_limit_xy").as_double();
    velocity_limits_.acceleration_yaw =
        node->get_parameter("acceleration_limit_yaw").as_double();

    const auto nominal = node->get_parameter("nominal_joint_positions").as_double_array();
    if (nominal.size() != kJointCount) {
      throw std::invalid_argument("nominal_joint_positions must contain 12 values");
    }
    std::copy(nominal.begin(), nominal.end(), nominal_joint_positions_.begin());
    const auto stand_up =
        node->get_parameter("stand_up_joint_positions").as_double_array();
    if (stand_up.size() != kJointCount) {
      throw std::invalid_argument("stand_up_joint_positions must contain 12 values");
    }
    std::copy(
        stand_up.begin(), stand_up.end(), stand_up_joint_positions_.begin());
    const auto passive =
        node->get_parameter("passive_joint_positions").as_double_array();
    if (passive.size() != kJointCount) {
      throw std::invalid_argument("passive_joint_positions must contain 12 values");
    }
    std::copy(passive.begin(), passive.end(), passive_joint_positions_.begin());

    const std::string share =
        ament_index_cpp::get_package_share_directory("custom_dog_control");
    task_file_ = node->get_parameter("task_file").as_string();
    reference_file_ = node->get_parameter("reference_file").as_string();
    urdf_file_ = node->get_parameter("urdf_file").as_string();
    if (task_file_.empty()) {
      task_file_ = share + "/config/nmpc/task.info";
    }
    if (reference_file_.empty()) {
      reference_file_ = share + "/config/nmpc/reference.info";
    }

    const std::string robot_description =
        node->get_parameter("robot_description").as_string();
    if (!robot_description.empty()) {
      generated_urdf_file_ =
          "/tmp/custom_dog_control_robot_description_" + std::to_string(::getpid()) + ".urdf";
      std::ofstream urdf_stream(generated_urdf_file_);
      if (!urdf_stream) {
        throw std::runtime_error("cannot create temporary URDF from robot_description");
      }
      urdf_stream << robot_description;
      urdf_stream.close();
      urdf_file_ = generated_urdf_file_;
    }
    if (urdf_file_.empty()) {
      const std::string description_share =
          ament_index_cpp::get_package_share_directory("custom_dog_description");
      urdf_file_ = description_share + "/urdf/custom_dog.urdf";
    }

    NmpcBackendConfig backend_config;
    backend_config.task_file = task_file_;
    backend_config.reference_file = reference_file_;
    backend_config.urdf_file = urdf_file_;
    backend_config.frequency_hz =
        node->get_parameter("mpc_frequency_hz").as_double();
    backend_config.target_horizon_s = 1.0;
    backend_config.nominal_height_m = nominal_height_m_;
    backend_config.max_hip_angle_rad =
        node->get_parameter("locomotion_max_hip_angle_rad").as_double();
    backend_config.joint_limit_margin_rad =
        node->get_parameter("locomotion_joint_limit_margin_rad").as_double();
    if (backend_config.max_hip_angle_rad <= 0.0 ||
        backend_config.joint_limit_margin_rad < 0.0) {
      throw std::invalid_argument("locomotion joint safety domain is invalid");
    }
    backend_config.nominal_joint_positions = nominal_joint_positions_;

    backend_ = std::make_unique<NmpcBackend>();
    const auto validation = backend_->Configure(backend_config);
    if (!validation.ok) {
      throw std::runtime_error(validation.Summary());
    }
    estimator_ = std::make_unique<KinematicStateEstimator>(
        backend_->pinocchioInterface(), backend_->modelInfo());
    estimator_->Reset(nominal_height_m_);

    SafetyLimits safety_limits;
    const double policy_timeout_cycles =
        IsRealHardware()
            ? 2.0
            : node->get_parameter("simulation_policy_timeout_cycles")
                  .as_double();
    if (policy_timeout_cycles < 2.0) {
      throw std::invalid_argument(
          "simulation_policy_timeout_cycles must be at least 2.0");
    }
    safety_limits.max_policy_age_s =
        policy_timeout_cycles / backend_config.frequency_hz;
    safety_limits.joint_position_tolerance_rad =
        node->get_parameter("joint_limit_tolerance_rad").as_double();
    if (safety_limits.joint_position_tolerance_rad < 0.0) {
      throw std::invalid_argument("joint_limit_tolerance_rad must be non-negative");
    }
    const auto& model = backend_->pinocchioInterface().getModel();
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto joint_id = model.getJointId(joints_[i]);
      const auto q_index = model.joints[joint_id].idx_q();
      const auto v_index = model.joints[joint_id].idx_v();
      safety_limits.lower_position[i] = model.lowerPositionLimit(q_index);
      safety_limits.upper_position[i] = model.upperPositionLimit(q_index);
      safety_limits.effort_limit[i] = model.effortLimit(v_index);
      if (nominal_joint_positions_[i] <= safety_limits.lower_position[i] ||
          nominal_joint_positions_[i] >= safety_limits.upper_position[i]) {
        throw std::invalid_argument("nominal joint position violates URDF limit");
      }
      if (stand_up_joint_positions_[i] <= safety_limits.lower_position[i] ||
          stand_up_joint_positions_[i] >= safety_limits.upper_position[i]) {
        throw std::invalid_argument("stand-up joint position violates URDF limit");
      }
      if (passive_joint_positions_[i] <= safety_limits.lower_position[i] ||
          passive_joint_positions_[i] >= safety_limits.upper_position[i]) {
        throw std::invalid_argument("passive joint position violates URDF limit");
      }
    }
    safety_monitor_ = std::make_unique<SafetyMonitor>(safety_limits);

    imu_subscription_ = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&NmpcWbcController::ImuCallback, this, std::placeholders::_1));
    if (!IsRealHardware() && use_sim_ground_truth_) {
      ground_truth_subscription_ =
          node->create_subscription<nav_msgs::msg::Odometry>(
              "/ground_truth/odom", rclcpp::SensorDataQoS().keep_last(1),
              std::bind(
                  &NmpcWbcController::GroundTruthCallback, this,
                  std::placeholders::_1));
    }
    joy_subscription_ = node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&NmpcWbcController::JoyCallback, this, std::placeholders::_1));
    cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(1),
        std::bind(&NmpcWbcController::CmdVelCallback, this, std::placeholders::_1));

    mode_publisher_ = node->create_publisher<std_msgs::msg::String>(
        "~/control_mode", rclcpp::QoS(1).transient_local());
    contact_publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/contact_plan", 10);
    diagnostics_publisher_ =
        node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "~/diagnostics", 10);
    odom_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    imu_buffer_.writeFromNonRT(ImuSample{});
    ground_truth_buffer_.writeFromNonRT(EstimatedState{});
    joy_buffer_.writeFromNonRT(JoyInput{});
    cmd_vel_buffer_.writeFromNonRT(VelocityCommand{});
    policy_buffer_.writeFromNonRT(PolicySample{});
    RCLCPP_INFO(
        node->get_logger(), "Configured custom dog model: %s",
        validation.Summary().c_str());
    RCLCPP_INFO(
        node->get_logger(),
        "Algorithm backend: legged::LeggedInterface + legged::WeightedWbc "
        "(qiayuanl/legged_control a7f381c0367e98e31c01336e678eef47e304d40d)");
    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Configuration failed: %s", exception.what());
    backend_.reset();
    estimator_.reset();
    return controller_interface::CallbackReturn::ERROR;
  }
}

controller_interface::CallbackReturn NmpcWbcController::on_activate(
    const rclcpp_lifecycle::State&) {
  if (!ResolveInterfaceIndices()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  mode_ = OperatingMode::PASSIVE;
  stance_handoff_active_ = false;
  requires_recalibration_ = true;
  reset_calibration_pending_ = false;
  observation_time_ = 0.0;
  last_target_update_seconds_ = -1.0;
  target_command_was_moving_ = false;
  state_entered_seconds_ = get_node()->get_clock()->now().seconds();
  limited_command_ = {};
  consecutive_wbc_failures_ = 0;
  last_timed_policy_sequence_ = 0;
  gait_request_policy_sequence_ = 0;
  trot_enabled_ = false;
  gait_transition_ = GaitTransition::NONE;
  gait_condition_since_seconds_ = -1.0;
  control_timing_.Reset();
  io_timing_.Reset();
  mpc_timing_.Reset();
  if (safety_monitor_) {
    safety_monitor_->Reset();
  }
  backend_->Start();
  mode_publisher_->on_activate();
  contact_publisher_->on_activate();
  diagnostics_publisher_->on_activate();
  odom_publisher_->on_activate();
  WriteSafeCommand();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool NmpcWbcController::ResolveInterfaceIndices() {
  const auto find_command = [this](const std::string& name, std::size_t& index) {
    for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
      if (command_interfaces_[i].get_name() == name) {
        index = i;
        return true;
      }
    }
    RCLCPP_ERROR(get_node()->get_logger(), "Missing command interface: %s", name.c_str());
    return false;
  };
  const auto find_state = [this](const std::string& name, std::size_t& index) {
    for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
      if (state_interfaces_[i].get_name() == name) {
        index = i;
        return true;
      }
    }
    RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface: %s", name.c_str());
    return false;
  };

  const std::array<std::string, 5> real_command_types = {
      "position", "velocity", "effort", "kp", "kd"};
  const std::array<std::string, 5> real_state_types = {
      "position", "velocity", "effort", "temperature", "valid"};
  const std::array<std::string, 3> simulation_state_types = {
      "position", "velocity", "effort"};

  bool valid = true;
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (IsRealHardware()) {
      for (std::size_t type = 0; type < real_command_types.size(); ++type) {
        valid = find_command(
                    joints_[joint] + "/" + real_command_types[type],
                    command_interface_indices_[joint][type]) && valid;
      }
      for (std::size_t type = 0; type < real_state_types.size(); ++type) {
        valid = find_state(
                    joints_[joint] + "/" + real_state_types[type],
                    state_interface_indices_[joint][type]) && valid;
      }
    } else {
      valid = find_command(
                  joints_[joint] + "/effort",
                  command_interface_indices_[joint][0]) && valid;
      for (std::size_t type = 0; type < simulation_state_types.size(); ++type) {
        valid = find_state(
                    joints_[joint] + "/" + simulation_state_types[type],
                    state_interface_indices_[joint][type]) && valid;
      }
    }
  }

  if (IsRealHardware()) {
    const std::array<std::string, 2> system_commands = {
        "custom_dog/calibrate", "custom_dog/emergency_stop"};
    const std::array<std::string, 5> system_states = {
        "custom_dog/calibrated", "custom_dog/communication_ok",
        "custom_dog/physical_estop", "custom_dog/io_period_ms",
        "custom_dog/io_timeout_count"};
    for (std::size_t i = 0; i < system_commands.size(); ++i) {
      valid = find_command(system_commands[i], system_command_interface_indices_[i]) && valid;
    }
    for (std::size_t i = 0; i < system_states.size(); ++i) {
      valid = find_state(system_states[i], system_state_interface_indices_[i]) && valid;
    }
  }
  return valid;
}

controller_interface::CallbackReturn NmpcWbcController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  mode_ = OperatingMode::FAULT;
  WriteSafeCommand();
  if (backend_) {
    backend_->Stop();
  }
  mode_publisher_->on_deactivate();
  contact_publisher_->on_deactivate();
  diagnostics_publisher_->on_deactivate();
  odom_publisher_->on_deactivate();
  return controller_interface::CallbackReturn::SUCCESS;
}

void NmpcWbcController::ImuCallback(
    const sensor_msgs::msg::Imu::SharedPtr message) {
  ImuSample sample;
  sample.orientation_wxyz = {
      message->orientation.w, message->orientation.x,
      message->orientation.y, message->orientation.z};
  sample.angular_velocity = {
      message->angular_velocity.x, message->angular_velocity.y,
      message->angular_velocity.z};
  sample.linear_acceleration = {
      message->linear_acceleration.x, message->linear_acceleration.y,
      message->linear_acceleration.z};
  const rclcpp::Time sensor_stamp(message->header.stamp);
  sample.stamp_seconds =
      sensor_stamp.nanoseconds() > 0
          ? sensor_stamp.seconds()
          : get_node()->get_clock()->now().seconds();
  sample.valid = std::isfinite(message->orientation.w) &&
                 std::isfinite(message->linear_acceleration.x);
  imu_buffer_.writeFromNonRT(sample);
}

void NmpcWbcController::GroundTruthCallback(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  EstimatedState sample;
  sample.position = {
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z};
  Eigen::Quaterniond orientation(
      message->pose.pose.orientation.w,
      message->pose.pose.orientation.x,
      message->pose.pose.orientation.y,
      message->pose.pose.orientation.z);
  if (std::isfinite(orientation.norm()) && orientation.norm() > 1e-6) {
    orientation.normalize();
    sample.euler_zyx = EulerZyxFromRotation(orientation.toRotationMatrix());
  }
  sample.velocity_world = {
      message->twist.twist.linear.x,
      message->twist.twist.linear.y,
      message->twist.twist.linear.z};
  sample.angular_velocity_world = {
      message->twist.twist.angular.x,
      message->twist.twist.angular.y,
      message->twist.twist.angular.z};
  sample.stamp_seconds = get_node()->get_clock()->now().seconds();
  sample.valid = sample.position.allFinite() &&
                 sample.euler_zyx.allFinite() &&
                 sample.velocity_world.allFinite() &&
                 sample.angular_velocity_world.allFinite();
  ground_truth_buffer_.writeFromNonRT(sample);
}

void NmpcWbcController::JoyCallback(
    const sensor_msgs::msg::Joy::SharedPtr message) {
  JoyInput input;
  input.stamp_seconds = get_node()->get_clock()->now().seconds();
  input.velocity.vx = Axis(*message, 1) * velocity_limits_.vx;
  input.velocity.vy = LegacyJoyLateralToRep103(
      Axis(*message, 0) * velocity_limits_.vy, legacy_joy_y_right_);
  input.velocity.yaw = Axis(*message, 3) * velocity_limits_.yaw;
  input.velocity.stamp_seconds = input.stamp_seconds;
  input.velocity.active = true;

  if (Button(*message, 3)) {
    input.requested_mode = RequestedMode::ESTOP;
  } else if (Button(*message, 1)) {
    input.requested_mode = RequestedMode::PASSIVE;
  } else if (Button(*message, 7)) {
    input.requested_mode = RequestedMode::CALIBRATE_AND_STAND;
  } else if (Button(*message, 2)) {
    input.requested_mode = RequestedMode::TROT;
  } else if (Button(*message, 0)) {
    input.requested_mode = RequestedMode::STANCE;
  }
  const double magnitude =
      std::max({std::abs(input.velocity.vx), std::abs(input.velocity.vy),
                std::abs(input.velocity.yaw)});
  // Mode buttons must not make a zero-valued joystick override /cmd_vel.
  // The request is consumed independently through requested_mode and its stamp.
  input.active = magnitude > 0.01;
  joy_buffer_.writeFromNonRT(input);
}

void NmpcWbcController::CmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr message) {
  VelocityCommand command;
  command.vx = message->linear.x;
  command.vy = message->linear.y;
  command.yaw = message->angular.z;
  command.stamp_seconds = get_node()->get_clock()->now().seconds();
  command.active = true;
  cmd_vel_buffer_.writeFromNonRT(ClampVelocity(command, velocity_limits_));
}

void NmpcWbcController::TransitionTo(
    OperatingMode next, double now_seconds) {
  if (mode_ == next) {
    return;
  }
  const OperatingMode previous = mode_;
  RCLCPP_INFO(
      get_node()->get_logger(), "Control mode %s -> %s",
      ToString(mode_).data(), ToString(next).data());
  mode_ = next;
  state_entered_seconds_ = now_seconds;
  if (next == OperatingMode::STAND_UP) {
    consecutive_wbc_failures_ = 0;
    stand_start_positions_ = joints_state_.position;
  } else if (next == OperatingMode::MPC_STANCE) {
    // The fixed-pose blend is only the position-control-to-WBC handoff.
    // Reapplying it after Trot would drag four planted feet back to the
    // original stand posture and can overturn the robot after a yaw command.
    stance_handoff_active_ = previous == OperatingMode::STAND_UP;
    consecutive_wbc_failures_ = 0;
    // Arm locomotion after the explicit calibration and stand-up sequence.
    // Zero velocity is still handled as MPC_STANCE by the gait supervisor.
    trot_enabled_ = true;
    backend_->RequestGait(false);
    backend_->SetVelocityCommand(VelocityCommand{});
    last_target_update_seconds_ = now_seconds;
    target_command_was_moving_ = false;
    stance_anchor_xy_ = {estimate_.position.x(), estimate_.position.y()};
  } else if (next == OperatingMode::FAULT) {
    requires_recalibration_ = true;
    trot_enabled_ = false;
    gait_transition_ = GaitTransition::NONE;
    gait_condition_since_seconds_ = -1.0;
  }
}

void NmpcWbcController::ReadHardwareState() {
  if (IsRealHardware()) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      joints_state_.position[i] = state_interfaces_[state_interface_indices_[i][0]].get_value();
      joints_state_.velocity[i] = state_interfaces_[state_interface_indices_[i][1]].get_value();
      joints_state_.effort[i] = state_interfaces_[state_interface_indices_[i][2]].get_value();
      joints_state_.temperature[i] = state_interfaces_[state_interface_indices_[i][3]].get_value();
      joints_state_.valid[i] = state_interfaces_[state_interface_indices_[i][4]].get_value();
    }
    calibrated_ = state_interfaces_[system_state_interface_indices_[0]].get_value() > 0.5;
    communication_ok_ = state_interfaces_[system_state_interface_indices_[1]].get_value() > 0.5;
    physical_estop_ = state_interfaces_[system_state_interface_indices_[2]].get_value() > 0.5;
    io_period_ms_ = state_interfaces_[system_state_interface_indices_[3]].get_value();
    io_timeout_count_ = state_interfaces_[system_state_interface_indices_[4]].get_value();
    consecutive_io_failures_ =
        communication_ok_ ? 0 : consecutive_io_failures_ + 1;
  } else {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      joints_state_.position[i] = state_interfaces_[state_interface_indices_[i][0]].get_value();
      joints_state_.velocity[i] = state_interfaces_[state_interface_indices_[i][1]].get_value();
      joints_state_.effort[i] = state_interfaces_[state_interface_indices_[i][2]].get_value();
      joints_state_.temperature[i] = 0.0;
      joints_state_.valid[i] = 1.0;
    }
    calibrated_ = true;
    communication_ok_ = true;
    physical_estop_ = false;
    consecutive_io_failures_ = 0;
  }
}

VelocityCommand NmpcWbcController::SelectVelocityCommand(
    double now_seconds, double dt) {
  const JoyInput joy = *joy_buffer_.readFromRT();
  const VelocityCommand cmd_vel = *cmd_vel_buffer_.readFromRT();
  VelocityCommand target;
  if (joy.active && now_seconds - joy.stamp_seconds <= joy_timeout_s_) {
    target = joy.velocity;
  } else if (cmd_vel.active &&
             now_seconds - cmd_vel.stamp_seconds <= command_timeout_s_) {
    target = cmd_vel;
  }
  target = ClampVelocity(target, velocity_limits_);
  limited_command_ = SlewVelocity(limited_command_, target, velocity_limits_, dt);
  limited_command_.stamp_seconds = now_seconds;
  // A source timeout or zero command starts a controlled deceleration; it
  // must not deactivate locomotion while the slew-limited command is still
  // moving. Otherwise the supervisor inserts STANCE at running speed.
  limited_command_.active =
      target.active || !IsBelowTrotExitThreshold(limited_command_);
  return limited_command_;
}

void NmpcWbcController::UpdateLocomotionSupervisor(
    double now_seconds, const VelocityCommand& command, const JoyInput& joy,
    const PolicySample& policy, double policy_age_seconds) {
  const RequestedMode request =
      now_seconds - joy.stamp_seconds <= joy_timeout_s_
          ? joy.requested_mode
          : RequestedMode::NONE;
  if (request == RequestedMode::TROT &&
      (mode_ == OperatingMode::MPC_STANCE ||
       mode_ == OperatingMode::MPC_TROT)) {
    trot_enabled_ = true;
  } else if (request == RequestedMode::STANCE) {
    trot_enabled_ = false;
  }

  if (mode_ != OperatingMode::MPC_STANCE &&
      mode_ != OperatingMode::MPC_TROT) {
    gait_transition_ = GaitTransition::NONE;
    gait_condition_since_seconds_ = -1.0;
    return;
  }

  const bool policy_fresh =
      policy.valid && backend_->solverHealthy() &&
      policy_age_seconds <= safety_monitor_->limits().max_policy_age_s;

  if (gait_transition_ == GaitTransition::STARTING_TROT) {
    if (!trot_enabled_ || !ExceedsTrotEntryThreshold(command)) {
      backend_->RequestGait(false);
      gait_transition_ = GaitTransition::NONE;
      gait_condition_since_seconds_ = -1.0;
    } else if (policy_fresh &&
               policy.sequence > gait_request_policy_sequence_ &&
               IsTrotMode(policy.mode)) {
      gait_transition_ = GaitTransition::NONE;
      gait_condition_since_seconds_ = -1.0;
      TransitionTo(OperatingMode::MPC_TROT, now_seconds);
    } else {
      backend_->RequestGait(true);
    }
    return;
  }

  if (gait_transition_ == GaitTransition::STOPPING_TROT) {
    if (policy_fresh &&
        policy.sequence > gait_request_policy_sequence_ &&
        policy.mode == kStanceMode) {
      gait_transition_ = GaitTransition::NONE;
      gait_condition_since_seconds_ = -1.0;
      TransitionTo(OperatingMode::MPC_STANCE, now_seconds);
    }
    return;
  }

  if (mode_ == OperatingMode::MPC_STANCE) {
    if (!trot_enabled_ || !ExceedsTrotEntryThreshold(command) ||
        !policy_fresh) {
      gait_condition_since_seconds_ = -1.0;
      return;
    }
    if (gait_condition_since_seconds_ < 0.0) {
      gait_condition_since_seconds_ = now_seconds;
    }
    if (now_seconds - gait_condition_since_seconds_ >= trot_entry_dwell_s_) {
      backend_->RequestGait(true);
      gait_request_policy_sequence_ = policy.sequence;
      gait_transition_ = GaitTransition::STARTING_TROT;
      gait_condition_since_seconds_ = -1.0;
    }
    return;
  }

  const bool stop_requested =
      !trot_enabled_ || IsBelowTrotExitThreshold(command) || !command.active;
  if (!stop_requested) {
    backend_->RequestGait(true);
    gait_condition_since_seconds_ = -1.0;
    return;
  }
  if (gait_condition_since_seconds_ < 0.0) {
    gait_condition_since_seconds_ = now_seconds;
  }
  if (now_seconds - gait_condition_since_seconds_ >= trot_exit_dwell_s_) {
    backend_->RequestGait(false);
    gait_request_policy_sequence_ = policy.sequence;
    gait_transition_ = GaitTransition::STOPPING_TROT;
    gait_condition_since_seconds_ = -1.0;
  }
}

void NmpcWbcController::WriteHybridCommand(
    const HybridJointCommand& command) {
  if (IsRealHardware()) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      command_interfaces_[command_interface_indices_[i][0]].set_value(command.position[i]);
      command_interfaces_[command_interface_indices_[i][1]].set_value(command.velocity[i]);
      command_interfaces_[command_interface_indices_[i][2]].set_value(command.effort[i]);
      command_interfaces_[command_interface_indices_[i][3]].set_value(command.kp[i]);
      command_interfaces_[command_interface_indices_[i][4]].set_value(command.kd[i]);
    }
    double calibration = mode_ == OperatingMode::CALIBRATION ? 1.0 : 0.0;
    if (reset_calibration_pending_) {
      calibration = -1.0;
      reset_calibration_pending_ = false;
    }
    command_interfaces_[system_command_interface_indices_[0]].set_value(calibration);
    command_interfaces_[system_command_interface_indices_[1]].set_value(
        mode_ == OperatingMode::FAULT ? 1.0 : 0.0);
  } else {
    const auto& effort_limits = safety_monitor_->limits().effort_limit;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double equivalent_effort =
          command.effort[i] +
          command.kp[i] * (command.position[i] - joints_state_.position[i]) +
          command.kd[i] * (command.velocity[i] - joints_state_.velocity[i]);
      const double effort_limit = std::max(0.0, effort_limits[i]);
      command_interfaces_[command_interface_indices_[i][0]].set_value(
          std::clamp(equivalent_effort, -effort_limit, effort_limit));
    }
  }
}

void NmpcWbcController::WriteSafeCommand(bool reset_calibration) {
  HybridJointCommand command;
  const bool hold_simulation_pose =
      !IsRealHardware() && mode_ == OperatingMode::PASSIVE &&
      simulation_passive_hold_;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (hold_simulation_pose) {
      command.position[i] = passive_joint_positions_[i];
      command.kp[i] =
          (i % 3 == 0) ? simulation_passive_hip_kp_
                       : simulation_passive_leg_kp_;
      command.kd[i] = simulation_passive_kd_;
    } else {
      command.position[i] = std::isfinite(joints_state_.position[i])
                                ? joints_state_.position[i]
                                : 0.0;
      command.kd[i] = safe_damping_;
    }
  }
  reset_calibration_pending_ = reset_calibration_pending_ || reset_calibration;
  WriteHybridCommand(command);
}

bool NmpcWbcController::ApplyStateMachine(
    double now_seconds, double dt, const JoyInput& joy,
    const PolicySample& policy, bool policy_available,
    HybridJointCommand& command, WbcOutput& wbc) {
  const RequestedMode request =
      now_seconds - joy.stamp_seconds <= joy_timeout_s_
          ? joy.requested_mode
          : RequestedMode::NONE;

  if (request == RequestedMode::ESTOP) {
    TransitionTo(OperatingMode::FAULT, now_seconds);
  }
  if (mode_ == OperatingMode::FAULT) {
    if (request == RequestedMode::PASSIVE) {
      safety_monitor_->Reset();
      reset_calibration_pending_ = IsRealHardware();
      requires_recalibration_ = true;
      TransitionTo(OperatingMode::PASSIVE, now_seconds);
    }
    return true;
  }

  if (request == RequestedMode::PASSIVE) {
    TransitionTo(OperatingMode::PASSIVE, now_seconds);
  }
  if (mode_ == OperatingMode::PASSIVE &&
      (request == RequestedMode::CALIBRATE_AND_STAND ||
       request == RequestedMode::STANCE ||
       request == RequestedMode::TROT)) {
    if (IsRealHardware() && (requires_recalibration_ || !calibrated_)) {
      TransitionTo(OperatingMode::CALIBRATION, now_seconds);
    } else {
      TransitionTo(OperatingMode::STAND_UP, now_seconds);
    }
  }

  if (mode_ == OperatingMode::CALIBRATION) {
    if (calibrated_) {
      requires_recalibration_ = false;
      TransitionTo(OperatingMode::STAND_UP, now_seconds);
    }
    return true;
  }

  if (mode_ == OperatingMode::PASSIVE) {
    return true;
  }

  if (mode_ == OperatingMode::STAND_UP) {
    const double elapsed = now_seconds - state_entered_seconds_;
    const double duration = std::max(0.1, stand_up_duration_s_);
    // Match unitree_guide FixedStand: interpolate every joint from the
    // measured entry pose to the fixed standing pose with position PD.
    const double phase = SmoothStep(elapsed / duration);
    const double settle_phase = SmoothStep(
        (elapsed - stand_up_duration_s_) /
        std::max(0.1, stand_up_settle_duration_s_));
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const std::size_t joint_in_leg = i % kJointsPerLeg;
      command.position[i] =
          (1.0 - phase) * stand_start_positions_[i] +
          phase * stand_up_joint_positions_[i];
      command.velocity[i] = 0.0;
      command.effort[i] = 0.0;
      const double initial_kp =
          joint_in_leg == 0
              ? stand_up_hip_kp_
              : (joint_in_leg == 1 ? stand_up_leg_kp_ : stand_up_calf_kp_);
      const double settled_kp =
          joint_in_leg == 0
              ? stand_up_settle_hip_kp_
              : (joint_in_leg == 1 ? stand_up_settle_leg_kp_
                                   : stand_up_settle_calf_kp_);
      command.kp[i] =
          (1.0 - settle_phase) * initial_kp + settle_phase * settled_kp;
      command.kd[i] =
          (1.0 - settle_phase) * stand_up_kd_ +
          settle_phase * stand_up_settle_kd_;
    }
    bool joints_settled = true;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      joints_settled =
          joints_settled &&
          std::abs(joints_state_.position[i] - stand_up_joint_positions_[i]) <=
              stand_up_position_tolerance_rad_ &&
          std::abs(joints_state_.velocity[i]) <=
              stand_up_velocity_tolerance_rad_s_;
    }
    if (elapsed >= stand_up_duration_s_ + stand_up_settle_duration_s_ &&
        joints_settled &&
        estimate_.velocity_world.norm() <=
            stand_up_base_velocity_tolerance_m_s_ &&
        policy_available) {
      TransitionTo(OperatingMode::MPC_STANCE, now_seconds);
    }
    return true;
  }

  if (!policy_available || measured_rbd_state_.size() == 0) {
    return false;
  }
  wbc = backend_->ComputeWbc(policy, measured_rbd_state_, dt);
  if (!wbc.valid) {
    ++consecutive_wbc_failures_;
    if (consecutive_wbc_failures_ >= max_consecutive_wbc_failures_) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "WBC failed for %d consecutive control cycles; entering FAULT",
          consecutive_wbc_failures_);
      TransitionTo(OperatingMode::FAULT, now_seconds);
      return false;
    }

    // A single active-set failure must not drop a standing robot. Hold the
    // proven position-controlled stance while NMPC/WBC recovers next cycle.
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const std::size_t joint_in_leg = i % kJointsPerLeg;
      command.position[i] = stand_up_joint_positions_[i];
      command.velocity[i] = 0.0;
      command.effort[i] = 0.0;
      command.kp[i] =
          joint_in_leg == 0
              ? stand_up_settle_hip_kp_
              : (joint_in_leg == 1 ? stand_up_settle_leg_kp_
                                   : stand_up_settle_calf_kp_);
      command.kd[i] = stand_up_settle_kd_;
    }
    return true;
  }
  consecutive_wbc_failures_ = 0;
  command = wbc.command;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    // Match legged_control's hybrid command: optimized position/velocity,
    // low joint impedance, and WBC feed-forward torque.
    command.kp[i] =
        i % kJointsPerLeg == 0 ? wbc_hip_stiffness_ : wbc_joint_stiffness_;
    command.kd[i] = wbc_joint_damping_;
  }
  if (mode_ == OperatingMode::MPC_STANCE && stance_handoff_active_) {
    const double alpha = SmoothStep(
        (now_seconds - state_entered_seconds_) /
        std::max(0.1, handoff_duration_s_));
    for (std::size_t i = 0; i < kJointCount; ++i) {
      command.position[i] =
          (1.0 - alpha) * stand_up_joint_positions_[i] +
          alpha * nominal_joint_positions_[i];
      command.effort[i] *= alpha;
      const std::size_t joint_in_leg = i % kJointsPerLeg;
      const double stand_kp =
          joint_in_leg == 0
              ? stand_up_settle_hip_kp_
              : (joint_in_leg == 1 ? stand_up_settle_leg_kp_
                                   : stand_up_settle_calf_kp_);
      command.kp[i] =
          (1.0 - alpha) * stand_kp +
          alpha * (joint_in_leg == 0 ? wbc_hip_stiffness_
                                     : wbc_joint_stiffness_);
      command.kd[i] =
          (1.0 - alpha) * stand_up_settle_kd_ +
          alpha * wbc_joint_damping_;
    }
  }
  return true;
}

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
    const bool target_update_due =
        target_command_is_moving &&
        now_seconds - last_target_update_seconds_ >= 0.05;
    const double stance_anchor_error = std::hypot(
        estimate_.position.x() - stance_anchor_xy_[0],
        estimate_.position.y() - stance_anchor_xy_[1]);
    const bool stance_reanchor_due =
        mode_ == OperatingMode::MPC_STANCE && !target_command_is_moving &&
        stance_anchor_error >= stance_reanchor_distance_m_;
    // Hold the world-frame stance target during normal contact motion. If
    // accumulated foot slip exceeds the configured guard distance, re-anchor
    // once so NMPC does not build an unsafe horizontal recovery force.
    if (last_target_update_seconds_ < 0.0 || target_update_due ||
        stance_reanchor_due ||
        (target_command_was_moving_ && !target_command_is_moving)) {
      backend_->SetVelocityCommand(backend_command);
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

void NmpcWbcController::PublishDiagnostics(
    const rclcpp::Time& stamp, const EstimatedState& estimate,
    const PolicySample& policy, const WbcOutput& wbc) {
  if (last_diagnostics_seconds_ >= 0.0 &&
      stamp.seconds() - last_diagnostics_seconds_ < 0.10) {
    return;
  }
  last_diagnostics_seconds_ = stamp.seconds();

  std_msgs::msg::String mode_message;
  mode_message.data = std::string(ToString(mode_));
  mode_publisher_->publish(mode_message);

  std_msgs::msg::Float64MultiArray contact_message;
  const auto contacts = ContactFlags(policy.valid ? policy.mode : kStanceMode);
  contact_message.data.reserve(kLegCount);
  for (const bool contact : contacts) {
    contact_message.data.push_back(contact ? 1.0 : 0.0);
  }
  contact_publisher_->publish(contact_message);

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "custom_dog_control/nmpc_wbc";
  status.hardware_id = IsRealHardware() ? "custom_dog_rs485" : "gazebo";
  status.level =
      mode_ == OperatingMode::FAULT
          ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
          : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message =
      mode_ == OperatingMode::FAULT && safety_monitor_->faulted()
          ? safety_monitor_->reason()
          : std::string(ToString(mode_));
  const auto control_timing = control_timing_.Snapshot();
  const auto io_timing = io_timing_.Snapshot();
  const auto mpc_timing = mpc_timing_.Snapshot();
  double wbc_tau_max_abs = 0.0;
  for (const double effort : wbc.command.effort) {
    wbc_tau_max_abs = std::max(wbc_tau_max_abs, std::abs(effort));
  }
  status.values = {
      KeyValue("mode", std::string(ToString(mode_))),
      KeyValue("trot_enabled", trot_enabled_ ? "true" : "false"),
      KeyValue(
          "gait_transition",
          gait_transition_ == GaitTransition::STARTING_TROT
              ? "STARTING_TROT"
              : (gait_transition_ == GaitTransition::STOPPING_TROT
                     ? "STOPPING_TROT"
                     : "NONE")),
      KeyValue("policy_mode", std::to_string(policy.mode)),
      KeyValue("nmpc_backend", "legged::LeggedInterface"),
      KeyValue("wbc_backend", "legged::WeightedWbc"),
      KeyValue(
          "legged_control_revision",
          "a7f381c0367e98e31c01336e678eef47e304d40d"),
      KeyValue("control_period_ms", Number(control_period_ms_)),
      KeyValue("control_period_mean_ms", Number(control_timing.mean)),
      KeyValue("control_period_p95_ms", Number(control_timing.p95)),
      KeyValue("control_period_p99_ms", Number(control_timing.p99)),
      KeyValue("io_period_ms", Number(io_period_ms_)),
      KeyValue("io_period_p99_ms", Number(io_timing.p99)),
      KeyValue("io_timeout_count", Number(io_timeout_count_)),
      KeyValue("communication_ok", communication_ok_ ? "true" : "false"),
      KeyValue("base_x_m", Number(estimate.position.x())),
      KeyValue("base_y_m", Number(estimate.position.y())),
      KeyValue("base_z_m", Number(estimate.position.z())),
      KeyValue("base_yaw_rad", Number(estimate.euler_zyx.x())),
      KeyValue("base_roll_rad", Number(estimate.euler_zyx.z())),
      KeyValue("base_pitch_rad", Number(estimate.euler_zyx.y())),
      KeyValue("base_vx_mps", Number(estimate.velocity_world.x())),
      KeyValue("base_vy_mps", Number(estimate.velocity_world.y())),
      KeyValue("base_vz_mps", Number(estimate.velocity_world.z())),
      KeyValue("requested_vx_mps", Number(limited_command_.vx)),
      KeyValue("requested_vy_mps", Number(limited_command_.vy)),
      KeyValue("requested_yaw_rad_s", Number(limited_command_.yaw)),
      KeyValue("governed_vx_mps", Number(governed_command_.vx)),
      KeyValue("governed_vy_mps", Number(governed_command_.vy)),
      KeyValue("governed_yaw_rad_s", Number(governed_command_.yaw)),
      KeyValue("fr_hip_position_rad", Number(joints_state_.position[0])),
      KeyValue("fr_thigh_position_rad", Number(joints_state_.position[1])),
      KeyValue("fr_calf_position_rad", Number(joints_state_.position[2])),
      KeyValue("fr_hip_velocity_rad_s", Number(joints_state_.velocity[0])),
      KeyValue("fr_thigh_velocity_rad_s", Number(joints_state_.velocity[1])),
      KeyValue("fr_calf_velocity_rad_s", Number(joints_state_.velocity[2])),
      KeyValue("mpc_policy_age_s", Number(backend_->policyAgeSeconds(stamp.seconds()))),
      KeyValue("mpc_solver_healthy", backend_->solverHealthy() ? "true" : "false"),
      KeyValue("mpc_last_error", backend_->lastError()),
      KeyValue("mpc_solve_ms", Number(backend_->lastSolveTimeMs())),
      KeyValue("mpc_solve_mean_ms", Number(mpc_timing.mean)),
      KeyValue("mpc_solve_p95_ms", Number(mpc_timing.p95)),
      KeyValue("mpc_solve_p99_ms", Number(mpc_timing.p99)),
      KeyValue("wbc_solve_ms", Number(wbc.solve_time_ms)),
      KeyValue("wbc_valid", wbc.valid ? "true" : "false"),
      KeyValue(
          "wbc_consecutive_failures",
          std::to_string(consecutive_wbc_failures_)),
      KeyValue("wbc_tau_max_abs", Number(wbc_tau_max_abs)),
      KeyValue("wbc_tau_fr_hip", Number(wbc.command.effort[0])),
      KeyValue("wbc_tau_fr_thigh", Number(wbc.command.effort[1])),
      KeyValue("wbc_tau_fr_calf", Number(wbc.command.effort[2])),
      KeyValue("wbc_equality_residual", Number(wbc.equality_residual)),
      KeyValue("wbc_inequality_violation", Number(wbc.inequality_violation)),
      KeyValue("policy_fz_fr", Number(policy.input[2])),
      KeyValue("policy_fz_fl", Number(policy.input[5])),
      KeyValue("policy_fz_rr", Number(policy.input[8])),
      KeyValue("policy_fz_rl", Number(policy.input[11]))};
  diagnostics.status.push_back(std::move(status));
  diagnostics_publisher_->publish(diagnostics);

  if (!estimate.valid) {
    return;
  }
  const Eigen::Quaterniond quaternion = QuaternionFromZyx(estimate.euler_zyx);
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base";
  odometry.pose.pose.position.x = estimate.position.x();
  odometry.pose.pose.position.y = estimate.position.y();
  odometry.pose.pose.position.z = estimate.position.z();
  odometry.pose.pose.orientation.w = quaternion.w();
  odometry.pose.pose.orientation.x = quaternion.x();
  odometry.pose.pose.orientation.y = quaternion.y();
  odometry.pose.pose.orientation.z = quaternion.z();
  const Eigen::Vector3d velocity_body =
      quaternion.inverse() * estimate.velocity_world;
  const Eigen::Vector3d angular_body =
      quaternion.inverse() * estimate.angular_velocity_world;
  odometry.twist.twist.linear.x = velocity_body.x();
  odometry.twist.twist.linear.y = velocity_body.y();
  odometry.twist.twist.linear.z = velocity_body.z();
  odometry.twist.twist.angular.x = angular_body.x();
  odometry.twist.twist.angular.y = angular_body.y();
  odometry.twist.twist.angular.z = angular_body.z();
  odom_publisher_->publish(odometry);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odometry.header;
  transform.child_frame_id = "base";
  transform.transform.translation.x = estimate.position.x();
  transform.transform.translation.y = estimate.position.y();
  transform.transform.translation.z = estimate.position.z();
  transform.transform.rotation = odometry.pose.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace custom_dog_control

PLUGINLIB_EXPORT_CLASS(
    custom_dog_control::NmpcWbcController,
    controller_interface::ControllerInterface)
