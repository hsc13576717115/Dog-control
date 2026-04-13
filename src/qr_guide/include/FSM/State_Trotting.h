#ifndef STATE_TROTTING_H
#define STATE_TROTTING_H

#include "FSM/FSMState.h"

// 小跑状态。
// 当前实现仍然是基于足端轨迹 + IK 的轻量 trotting，不重写原有策略核心。
class State_Trotting : public FSMState {
public:
    explicit State_Trotting(CtrlComponents* ctrlComp);
    ~State_Trotting() override = default;

    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;

    static constexpr double ANCHOR_UPDATE_T = 0.002;

private:
    // 运行时步态参数，由手柄输入在线调整。
    // 这里统一表达成“期望机体速度”，再映射到足端轨迹。
    struct MotionParams {
        double velocityX = 0.0;
        double velocityY = 0.0;
        double yawRate = 0.0;
        Vec2 joy = Vec2::Zero();
    } _motionParams;

    struct LimitParams {
        double maxAccelX = 3.0;
        double maxAccelY = 3.0;
        double maxAccelYaw = 5.0;
    } _limitParams;

    struct AccelLimitParams {
        double lastVelocityX = 0.0;
        double lastVelocityY = 0.0;
        double lastYawRate = 0.0;
    } _accelLimitParams;

    static constexpr double LIFT_H = 0.10;
    static constexpr double CYCLE_T = 0.35;
    static constexpr double HIP_JOINT_FIXED = 0.0;
    static constexpr double MOTION_EPS = 1e-3;
    static constexpr double LEG_PHASE[4] = {0.0, 0.5, 0.5, 0.0};

    double _startTime = 0.0;
    double _lastCommandUpdateTime = 0.0;
    int _transitionCount = 0;
    Vec12 _initMotorQ = Vec12::Zero();
    Vec3 _enterFootPos[4];
    Vec3 _nominalFootPos[4];
    Vec3 _lastLegQ[4];

    static double getTimeSec();
    Vec3 computeSwingFootTarget(int leg, double phase) const;
    Vec3 computeStanceFootTarget(int leg, double phase) const;
    Vec3 computeFrontFoothold(int leg) const;
    Vec3 computeRearFoothold(int leg) const;
    void processJoystickInput();
    void applyAccelerationLimits(double velocity_x, double velocity_y, double yaw_rate, double dt);
    bool hasActiveMotionCommand() const;
    void generateLegTrajectory(int leg, double masterT, double trans, Vec12& cmd, VecInt4& contact, Vec4& phase);
    void calculateIKAndApply(int leg, const Vec3& target_foot_in_hip, Vec12& cmd);
    Vec3 clampJointAngles(const Vec3& angles) const;
};

#endif  // STATE_TROTTING_H
