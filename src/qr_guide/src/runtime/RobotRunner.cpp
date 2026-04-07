#include "runtime/RobotRunner.h"

#include <array>
#include <iomanip>
#include <iostream>

#include "input/JoystickMapper.h"

namespace qr_guide {

namespace {

constexpr std::array<const char*, NumLeg> kLegNames = {"FR", "FL", "RR", "RL"};
constexpr auto kKinematicsPrintInterval = std::chrono::milliseconds(80);
constexpr double kJointPrintThresholdRad = 0.01;
constexpr double kFootPrintThresholdM = 0.002;

}  // namespace

RobotRunner::RobotRunner(std::shared_ptr<ControllerNode> controller_node,
                         std::unique_ptr<ControllerContext> context)
    : controller_node_(std::move(controller_node)),
      context_(std::move(context)),
      fsm_(std::make_unique<FSM>(context_.get())) {}

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
    }

    // FSM 在拿到最新输入、回读和估计结果之后再计算当前命令。
    fsm_->run();
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

}  // namespace qr_guide
