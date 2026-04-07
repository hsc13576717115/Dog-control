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
    struct MotionParams {
        double stepLenX = 0.0;
        double stepLenY = 0.0;
        double omega = 0.0;
        Vec2 joy = Vec2::Zero();
    } _motionParams;

    struct LimitParams {
        double maxAccelX = 0.25;
        double maxAccelY = 0.3;
    } _limitParams;

    struct AccelLimitParams {
        double lastStepLenX = 0.0;
        double lastStepLenY = 0.0;
    } _accelLimitParams;

    static constexpr double MAX_SWING_X = 0.50;
    static constexpr double MAX_SWING_Y = 0.10;
    static constexpr double MAX_OMEGA = 2.0;
    static constexpr double LIFT_H = 0.09;
    static constexpr double CYCLE_T = 0.4;
    static constexpr double HIP_JOINT_FIXED = 0.0;
    static constexpr double LEG_PHASE[4] = {0.0, 0.5, 0.5, 0.0};

    double _startTime = 0.0;
    int _transitionCount = 0;
    Vec12 _initMotorQ = Vec12::Zero();
    Vec3 _initFootPos[4];
    Vec3 _lastLegQ[4];

    static double getTimeSec();
    // 平移步态轨迹和纯转向轨迹。
    Vec3 cycloidTraj3D(double phase) const;
    Vec3 yawCycloidTraj3D(int leg, double phase, bool swing) const;
    void processJoystickInput();
    void applyAccelerationLimits(double& stepLenX, double& stepLenY, double dt);
    void generateLegTrajectory(int leg, double masterT, double trans, Vec12& cmd, VecInt4& contact, Vec4& phase);
    void calculateIKAndApply(int leg, const Vec3& target_foot_in_hip, Vec12& cmd);
    Vec3 clampJointAngles(const Vec3& angles) const;
};

#endif  // STATE_TROTTING_H
