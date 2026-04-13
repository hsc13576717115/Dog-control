#ifndef QR_GUIDE_CONTROL_HYBRIDSTANDCONTROLLER_H
#define QR_GUIDE_CONTROL_HYBRIDSTANDCONTROLLER_H

#include <array>

#include "common/mathTypes.h"
#include "config/RobotConfig.h"

class Estimator;
class LowlevelState;
class QuadrupedRobot;

namespace qr_guide {

// FixedStand 用的轻量站立态力位混合控制器。
// 当前只做 body z / roll / pitch 三个任务，并输出关节前馈力矩。
class HybridStandController {
public:
    enum class DebugStatus {
        IDLE,
        SUCCESS,
        MOTOR_FAULT,
        INVALID_STATE,
        INVALID_FEET,
        ALLOCATION_FAIL,
        INVALID_TORQUE,
    };

    struct DebugSnapshot {
        DebugStatus status = DebugStatus::IDLE;
        Vec3 rpy = Vec3::Zero();
        Vec3 gyro = Vec3::Zero();
        Vec3 position = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
        double z_des = 0.0;
        Vec3 body_task_wrench = Vec3::Zero();  // [Fz, Mx, My]
        Vec4 vertical_forces = Vec4::Zero();   // FR / FL / RR / RL
    };

    HybridStandController(const HybridStandParameters& parameters, double dt);

    void reset();

    const DebugSnapshot& debugSnapshot() const { return debug_snapshot_; }

    bool computeTorqueFeedforward(const std::array<Vec3, NumLeg>& target_feet_in_body,
                                  LowlevelState& low_state,
                                  Estimator* estimator,
                                  QuadrupedRobot& robot_model,
                                  double transition_scale,
                                  Vec12* tau_command);

private:
    bool hasMotorFault(const LowlevelState& low_state) const;
    bool computeBodyTaskWrench(const std::array<Vec3, NumLeg>& target_feet_in_body,
                               LowlevelState& low_state,
                               Estimator* estimator,
                               QuadrupedRobot& robot_model,
                               Vec3* body_task_wrench,
                               DebugSnapshot* debug_snapshot) const;
    bool allocateVerticalForces(const Vec34& current_feet_in_body,
                                const Vec3& body_task_wrench,
                                Vec4* vertical_forces) const;
    Vec12 applyJointTorqueSafety(const Vec12& tau_desired);

    HybridStandParameters parameters_;
    double dt_ = 0.002;
    Vec12 last_tau_command_ = Vec12::Zero();
    DebugSnapshot debug_snapshot_;
};

}  // namespace qr_guide

#endif  // QR_GUIDE_CONTROL_HYBRIDSTANDCONTROLLER_H
