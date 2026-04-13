#ifndef QR_GUIDE_RUNTIME_ROBOTRUNNER_H
#define QR_GUIDE_RUNTIME_ROBOTRUNNER_H

#include <chrono>
#include <memory>

#include "FSM/FSM.h"
#include "control/CtrlComponents.h"
#include "runtime/ControllerNode.h"
#include "runtime/VisualizationPublisher.h"

namespace qr_guide {

// 运行器负责把“输入采集 -> 硬件收发 -> 状态估计 -> FSM 输出”串成固定周期。
class RobotRunner {
public:
    RobotRunner(std::shared_ptr<ControllerNode> controller_node,
                std::unique_ptr<ControllerContext> context);

    int run(volatile sig_atomic_t* running_flag);
    bool step();

private:
    // 把 ROS IMU 消息写回控制器内部的 LowlevelState::imu。
    void applyImu(const sensor_msgs::msg::Imu& imu_msg) const;
    void handleCalibrationCompletion(bool was_calibrated_before_step);
    void maybePrintCalibrationKinematics(bool was_calibrated_before_step);
    void maybePrintEstimatorDebug(bool was_calibrated_before_step);

    std::shared_ptr<ControllerNode> controller_node_;
    std::unique_ptr<ControllerContext> context_;
    std::unique_ptr<FSM> fsm_;
    std::unique_ptr<VisualizationPublisher> visualization_publisher_;
    bool kinematics_debug_enabled_ = false;
    bool has_last_kinematics_snapshot_ = false;
    Vec34 last_printed_q_legs_ = Vec34::Zero();
    Vec34 last_printed_feet_in_hip_ = Vec34::Zero();
    std::chrono::steady_clock::time_point last_kinematics_print_time_ =
        std::chrono::steady_clock::time_point::min();
    bool estimator_debug_enabled_ = false;
    bool has_last_estimator_snapshot_ = false;
    Vec3 last_printed_estimated_position_ = Vec3::Zero();
    Vec3 last_printed_estimated_velocity_ = Vec3::Zero();
    VecInt4 last_printed_contact_ = VecInt4::Zero();
    Vec4 last_printed_phase_ = Vec4::Zero();
    Vec4 last_printed_feet_height_reference_ = Vec4::Zero();
    std::chrono::steady_clock::time_point last_estimator_print_time_ =
        std::chrono::steady_clock::time_point::min();
};

}  // namespace qr_guide

#endif  // QR_GUIDE_RUNTIME_ROBOTRUNNER_H
