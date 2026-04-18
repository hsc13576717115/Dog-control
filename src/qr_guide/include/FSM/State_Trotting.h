#ifndef STATE_TROTTING_H
#define STATE_TROTTING_H

#include <array>

#include "FSM/FSMState.h"

// 小跑状态。
// 第一阶段沿 unitree_guide 的思路把逻辑整理成：
// phase scheduler -> foothold calculator -> trajectory generator -> IK。
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
    struct MotionParams {
        double velocityX = 0.0;
        double velocityY = 0.0;
        double yawRate = 0.0;
        Vec2 joy = Vec2::Zero();
    } _motionParams;

    struct AccelLimitParams {
        double lastVelocityX = 0.0;
        double lastVelocityY = 0.0;
        double lastYawRate = 0.0;
    } _accelLimitParams;

    struct LegPhaseState {
        bool swing = false;
        double cyclePhase = 0.0;
        double segmentPhase = 0.0;
        double swingTime = 0.0;
        double stanceTime = 0.0;
        double remainingSwingTime = 0.0;
    };

    static constexpr double HIP_JOINT_FIXED = 0.0;
    static constexpr double MOTION_EPS = 1e-3;
    static constexpr double LEG_PHASE[4] = {0.0, 0.5, 0.5, 0.0};

    double _startTime = 0.0;
    double _lastCommandUpdateTime = 0.0;
    int _transitionCount = 0;
    qr_guide::JoyMappingParameters _joyMapping;
    qr_guide::TrotParameters _trotParams;
    Vec12 _initMotorQ = Vec12::Zero();
    std::array<Vec3, 4> _enterFootPos;
    std::array<Vec3, 4> _nominalFootPos;
    std::array<Vec3, 4> _lastLegQ;
    std::array<bool, 4> _prevSwingState;
    std::array<Vec3, 4> _swingStartFootPos;
    std::array<Vec3, 4> _swingTargetFootPos;
    std::array<Vec3, 4> _stanceStartFootPos;

    static double getTimeSec();
    Vec3 footHipToBodyFrame(int leg, const Vec3& foot_in_hip) const;
    Vec3 footBodyToHipFrame(int leg, const Vec3& foot_in_body) const;
    Vec3 rotateBodyPointYaw(const Vec3& point_body, double yaw_delta) const;
    LegPhaseState computeLegPhaseState(int leg, double masterT) const;
    Vec3 nominalFootBodyPosition(int leg) const;
    Vec3 projectBodyPointToRotationCircle(int leg, const Vec3& point_body) const;
    Vec3 computeSymmetricHalfStepShift(double stance_time) const;
    Vec3 computeTouchdownFootBodyTarget(int leg, double stance_time) const;
    Vec3 computePureRotationSwingFootTarget(int leg, double phase) const;
    Vec3 computeSwingFootTarget(int leg, double phase) const;
    Vec3 computeStanceFootTarget(int leg, const LegPhaseState& phase_state) const;
    Vec3 clampFootholdToWorkspace(int leg, const Vec3& foothold_in_hip) const;
    void updateLegPhaseAnchors(int leg, const LegPhaseState& phase_state);
    void syncAnchorsForStanding(const std::array<Vec3, 4>& foot_targets);
    void processJoystickInput();
    void applyAccelerationLimits(double velocity_x, double velocity_y, double yaw_rate, double dt);
    bool hasActiveMotionCommand() const;
    bool isPureRotationCommand() const;
    void generateLegTrajectory(int leg, double masterT, double trans, Vec12& cmd, VecInt4& contact, Vec4& phase);
    void calculateIKAndApply(int leg, const Vec3& target_foot_in_hip, Vec12& cmd);
    Vec3 clampJointAngles(const Vec3& angles) const;
};

#endif  // STATE_TROTTING_H
