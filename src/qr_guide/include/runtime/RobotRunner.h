#ifndef QR_GUIDE_RUNTIME_ROBOTRUNNER_H
#define QR_GUIDE_RUNTIME_ROBOTRUNNER_H

#include <chrono>
#include <memory>

#include "FSM/FSM.h"
#include "control/CtrlComponents.h"
#include "runtime/ControllerNode.h"

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
    void maybePrintCalibrationKinematics(bool was_calibrated_before_step);

    std::shared_ptr<ControllerNode> controller_node_;
    std::unique_ptr<ControllerContext> context_;
    std::unique_ptr<FSM> fsm_;
    bool kinematics_debug_enabled_ = false;
    bool has_last_kinematics_snapshot_ = false;
    Vec34 last_printed_q_legs_ = Vec34::Zero();
    Vec34 last_printed_feet_in_hip_ = Vec34::Zero();
    std::chrono::steady_clock::time_point last_kinematics_print_time_ =
        std::chrono::steady_clock::time_point::min();
};

}  // namespace qr_guide

#endif  // QR_GUIDE_RUNTIME_ROBOTRUNNER_H
