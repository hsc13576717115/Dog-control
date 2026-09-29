#include "custom_dog_control/controller/NmpcWbcController.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/multibody/model.hpp>

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
    // MPC 在后台按此频率求解；WBC 则随 controller_manager 的 update 周期执行。
    // SQP 迭代数和求解时域由 task.info 配置，两层控制频率不要混用。
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

    // 后端需要 URDF 文件路径；非空 robot_description 会生成进程临时文件，
    // 覆盖 urdf_file，并在析构时删除。两者都未提供时回退到描述包默认模型。
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

    // 先完成模型与求解器配置，再创建依赖相同模型布局的估计器和安全限值。
    // 配置失败直接拒绝生命周期切换，不能带着不完整映射进入 update。
    backend_ = std::make_unique<NmpcBackend>();
    const auto validation = backend_->Configure(backend_config);
    if (!validation.ok) {
      throw std::runtime_error(validation.Summary());
    }
    estimator_ = std::make_unique<KinematicStateEstimator>(
        backend_->pinocchioInterface(), backend_->modelInfo());
    estimator_->Reset(nominal_height_m_);

    SafetyLimits safety_limits;
    // 策略容许年龄按 MPC 周期换算为秒；实机固定两周期，仿真可配置更宽裕。
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
  // 激活只准备控制链路，仍从 PASSIVE 开始并要求标定；启动求解器本身
  // 不授予运动权限。接口索引必须在 controller_manager 分配句柄后解析。
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

controller_interface::CallbackReturn NmpcWbcController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  // 先把命令句柄切到安全输出，再等待求解线程退出；Stop 可能阻塞，
  // 因此放在生命周期路径，而不是周期控制路径。
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

}  // namespace custom_dog_control
