#include "runtime/RobotRunner.h"

#include <array>
#include <iomanip>
#include <iostream>

#include "input/JoystickMapper.h"

namespace qr_guide {

namespace {

constexpr std::array<const char*, NumLeg> kLegNames = {"FR", "FL", "RR", "RL"};
constexpr auto kKinematicsPrintInterval = std::chrono::milliseconds(80);
constexpr auto kEstimatorPrintInterval = std::chrono::milliseconds(120);
constexpr double kJointPrintThresholdRad = 0.01;
constexpr double kFootPrintThresholdM = 0.002;
constexpr double kEstimatorPositionThresholdM = 0.002;
constexpr double kEstimatorVelocityThresholdMps = 0.03;
constexpr double kEstimatorPhaseThreshold = 0.03;

}  // namespace

RobotRunner::RobotRunner(std::shared_ptr<ControllerNode> controller_node,
                         std::unique_ptr<ControllerContext> context)
    : controller_node_(std::move(controller_node)),
      context_(std::move(context)),
      fsm_(std::make_unique<FSM>(context_.get())),
      visualization_publisher_(
          std::make_unique<VisualizationPublisher>(controller_node_, context_->parameters)) {}

int RobotRunner::run(volatile sig_atomic_t* running_flag) {
    rclcpp::WallRate loop_rate(1.0 / context_->dt);
    while (*running_flag && rclcpp::ok()) {
        // 所有 ROS 回调都通过同一个节点进入，避免多节点分散管理。
        rclcpp::spin_some(controller_node_);
        if (!step()) {
            return -1;
        }
        loop_rate.sleep();
    }
    return 0;
}

bool RobotRunner::step() {
    const ControllerInputSnapshot snapshot = controller_node_->snapshot();
    applyImu(snapshot.imu);
    const bool was_calibrated_before_step = context_->isCalibrated();

    // 手柄先映射到低层状态，再由 IOSDK / FSM 统一消费。
    const UserInput user_input = JoystickMapper::Map(snapshot.joy, was_calibrated_before_step);
    context_->lowState->userCmd = user_input.command;
    context_->lowState->userValue = user_input.value;

    context_->sendRecv();
    maybePrintCalibrationKinematics(was_calibrated_before_step);
    if (context_->estimator) {
        context_->estimator->run();
        maybePrintEstimatorDebug(was_calibrated_before_step);
    }

    // FSM 在拿到最新输入、回读和估计结果之后再计算当前命令。
    fsm_->run();
    if (visualization_publisher_) {
        visualization_publisher_->publish(*context_, fsm_->currentStateLabel());
    }
    return true;
}

void RobotRunner::applyImu(const sensor_msgs::msg::Imu& imu_msg) const {
    IMU& imu = context_->lowState->imu;
    imu.quaternion[0] = static_cast<float>(imu_msg.orientation.w);
    imu.quaternion[1] = static_cast<float>(imu_msg.orientation.x);
    imu.quaternion[2] = static_cast<float>(imu_msg.orientation.y);
    imu.quaternion[3] = static_cast<float>(imu_msg.orientation.z);
    imu.gyroscope[0] = static_cast<float>(imu_msg.angular_velocity.x);
    imu.gyroscope[1] = static_cast<float>(imu_msg.angular_velocity.y);
    imu.gyroscope[2] = static_cast<float>(imu_msg.angular_velocity.z);
    imu.accelerometer[0] = static_cast<float>(imu_msg.linear_acceleration.x);
    imu.accelerometer[1] = static_cast<float>(imu_msg.linear_acceleration.y);
    imu.accelerometer[2] = static_cast<float>(imu_msg.linear_acceleration.z);
}

void RobotRunner::maybePrintCalibrationKinematics(bool was_calibrated_before_step) {
    if (!context_->isCalibrated()) {
        return;
    }

    bool force_print = false;
    if (!was_calibrated_before_step) {
        kinematics_debug_enabled_ = true;
        has_last_kinematics_snapshot_ = false;
        force_print = true;
        std::cout << "\n[CalibrationDebug] START 校准完成，已进入动态调试打印模式。" << std::endl;
        std::cout << "[CalibrationDebug] 下面先打印一帧当前姿态，之后只有当关节角或足端坐标发生明显变化时才会输出。"
                  << std::endl;
    }

    if (!kinematics_debug_enabled_) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!force_print &&
        last_kinematics_print_time_ != std::chrono::steady_clock::time_point::min() &&
        now - last_kinematics_print_time_ < kKinematicsPrintInterval) {
        return;
    }

    const Vec34 q_legs = context_->lowState->getQ();
    Vec34 feet_in_hip = Vec34::Zero();
    Vec34 feet_in_body = Vec34::Zero();
    for (int leg = 0; leg < NumLeg; ++leg) {
        feet_in_hip.col(leg) = context_->robotModel->forwardKinematics(q_legs.col(leg), leg, FrameType::HIP);
        feet_in_body.col(leg) = context_->robotModel->forwardKinematics(q_legs.col(leg), leg, FrameType::BODY);
    }

    const bool joint_changed =
        !has_last_kinematics_snapshot_ ||
        (q_legs - last_printed_q_legs_).cwiseAbs().maxCoeff() > kJointPrintThresholdRad;
    const bool foot_changed =
        !has_last_kinematics_snapshot_ ||
        (feet_in_hip - last_printed_feet_in_hip_).cwiseAbs().maxCoeff() > kFootPrintThresholdM;
    if (!force_print && !joint_changed && !foot_changed) {
        return;
    }

    last_kinematics_print_time_ = now;
    last_printed_q_legs_ = q_legs;
    last_printed_feet_in_hip_ = feet_in_hip;
    has_last_kinematics_snapshot_ = true;

    std::ios old_state(nullptr);
    old_state.copyfmt(std::cout);
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "[CalibrationDebug] ----------------------------------------" << std::endl;

    for (int leg = 0; leg < NumLeg; ++leg) {
        const Vec3 q_leg = q_legs.col(leg);
        const Vec3 foot_in_hip = feet_in_hip.col(leg);
        const Vec3 foot_in_body = feet_in_body.col(leg);

        std::cout << "[CalibrationDebug] " << kLegNames[leg]
                  << " q(rad)=[" << q_leg(0) << ", " << q_leg(1) << ", " << q_leg(2) << "]"
                  << " foot_hip(m)=[" << foot_in_hip(0) << ", " << foot_in_hip(1) << ", " << foot_in_hip(2) << "]"
                  << " foot_body(m)=[" << foot_in_body(0) << ", " << foot_in_body(1) << ", " << foot_in_body(2) << "]"
                  << std::endl;
    }

    std::cout.copyfmt(old_state);
}

void RobotRunner::maybePrintEstimatorDebug(bool was_calibrated_before_step) {
    if (!context_->isCalibrated() || !context_->estimator) {
        return;
    }

    bool force_print = false;
    if (!was_calibrated_before_step) {
        estimator_debug_enabled_ = true;
        has_last_estimator_snapshot_ = false;
        force_print = true;
        std::cout << "[EstimatorDebug] START 校准完成，已进入状态估计动态调试打印模式。" << std::endl;
        std::cout << "[EstimatorDebug] 会输出 position / velocity / contact / phase / foot_h_ref，后续只在明显变化时打印。"
                  << std::endl;
    }

    if (!estimator_debug_enabled_) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!force_print &&
        last_estimator_print_time_ != std::chrono::steady_clock::time_point::min() &&
        now - last_estimator_print_time_ < kEstimatorPrintInterval) {
        return;
    }

    const Vec3 position = context_->estimator->getPosition();
    const Vec3 velocity = context_->estimator->getVelocity();
    const VecInt4 contact = context_->contact;
    const Vec4 phase = context_->phase;
    const Vec4 feet_height_reference = context_->estimator->getFeetHeightReference();

    const bool position_changed =
        !has_last_estimator_snapshot_ ||
        (position - last_printed_estimated_position_).cwiseAbs().maxCoeff() > kEstimatorPositionThresholdM;
    const bool velocity_changed =
        !has_last_estimator_snapshot_ ||
        (velocity - last_printed_estimated_velocity_).cwiseAbs().maxCoeff() > kEstimatorVelocityThresholdMps;
    const bool contact_changed =
        !has_last_estimator_snapshot_ || (contact.array() != last_printed_contact_.array()).any();
    const bool phase_changed =
        !has_last_estimator_snapshot_ ||
        (phase - last_printed_phase_).cwiseAbs().maxCoeff() > kEstimatorPhaseThreshold;
    const bool feet_height_changed =
        !has_last_estimator_snapshot_ ||
        (feet_height_reference - last_printed_feet_height_reference_).cwiseAbs().maxCoeff() > 1e-6;

    if (!force_print && !position_changed && !velocity_changed && !contact_changed && !phase_changed &&
        !feet_height_changed) {
        return;
    }

    last_estimator_print_time_ = now;
    last_printed_estimated_position_ = position;
    last_printed_estimated_velocity_ = velocity;
    last_printed_contact_ = contact;
    last_printed_phase_ = phase;
    last_printed_feet_height_reference_ = feet_height_reference;
    has_last_estimator_snapshot_ = true;

    std::ios old_state(nullptr);
    old_state.copyfmt(std::cout);
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "[EstimatorDebug] position(m)=[" << position(0) << ", " << position(1) << ", " << position(2)
              << "] velocity(m/s)=[" << velocity(0) << ", " << velocity(1) << ", " << velocity(2) << "]"
              << " contact=[" << contact(0) << ", " << contact(1) << ", " << contact(2) << ", " << contact(3)
              << "] phase=[" << phase(0) << ", " << phase(1) << ", " << phase(2) << ", " << phase(3) << "]"
              << " foot_h_ref(m)=[" << feet_height_reference(0) << ", " << feet_height_reference(1) << ", "
              << feet_height_reference(2) << ", " << feet_height_reference(3) << "]" << std::endl;
    std::cout.copyfmt(old_state);
}

}  // namespace qr_guide
