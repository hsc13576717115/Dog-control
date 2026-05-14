/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef STATE_TROTTING_H
#define STATE_TROTTING_H

#include <array>
#include <memory>

#include "FSM/FSMState.h"

class Estimator;
class QuadrupedRobot;
class BalanceCtrl;

/**
 * @brief Trotting state with hybrid force/position control.
 *
 * Architecture:
 *   1. Gait scheduler (existing) -> contact flags + phase per leg
 *   2. Body PD controller -> desired CoM acceleration (ddPcd, dWbd)
 *   3. QP force allocator (BalanceCtrl) -> stance foot forces
 *   4. Swing trajectory PD -> swing foot forces
 *   5. Jacobian transpose -> joint torques
 *   6. IK -> joint positions/velocities for swing legs
 *   7. Compound mode output: tau + q + qd with per-leg gains
 */
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
    // ===== Gait scheduling (preserved from original) =====
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

    // ===== Force control (new) =====
    void calcBodyWrench();
    void calcFootForces();
    void calcJointTorques();
    void calcSwingQQd();
    void applyTorqueSafety();
    bool checkStepOrNot() const;

    // Pointers to shared components
    Estimator* _est = nullptr;
    BalanceCtrl* _balCtrl = nullptr;
    QuadrupedRobot* _robModel = nullptr;

    // Robot state
    Vec3 _posBody = Vec3::Zero();
    Vec3 _velBody = Vec3::Zero();
    double _yaw = 0.0;
    double _dYaw = 0.0;
    Vec34 _posFeetGlobal = Vec34::Zero();
    Vec34 _velFeetGlobal = Vec34::Zero();
    Vec34 _posFeet2BGlobal = Vec34::Zero();
    RotMat _B2G_RotMat = RotMat::Identity();
    RotMat _G2B_RotMat = RotMat::Identity();

    // Desired commands
    Vec3 _pcd = Vec3::Zero();
    Vec3 _vCmdGlobal = Vec3::Zero();
    Vec3 _vCmdBody = Vec3::Zero();
    double _yawCmd = 0.0;
    double _dYawCmd = 0.0;
    double _dYawCmdPast = 0.0;
    Vec3 _wCmdGlobal = Vec3::Zero();
    RotMat _Rd = RotMat::Identity();

    // Trajectory goals (global frame)
    Vec34 _posFeetGlobalGoal = Vec34::Zero();
    Vec34 _velFeetGlobalGoal = Vec34::Zero();

    // Body-frame goals
    Vec34 _posFeet2BGoal = Vec34::Zero();
    Vec34 _velFeet2BGoal = Vec34::Zero();

    // Control outputs
    Vec3 _ddPcd = Vec3::Zero();
    Vec3 _dWbd = Vec3::Zero();
    Vec34 _forceFeetGlobal = Vec34::Zero();
    Vec34 _forceFeetBody = Vec34::Zero();
    Vec34 _qGoal = Vec34::Zero();
    Vec34 _qdGoal = Vec34::Zero();
    Vec12 _tau = Vec12::Zero();
    Vec12 _tauPrev = Vec12::Zero();

    // Errors (for debug / step detection)
    Vec3 _posError = Vec3::Zero();
    Vec3 _velError = Vec3::Zero();

    // PD gains
    Mat3 _Kpp = Mat3::Zero();
    Mat3 _Kdp = Mat3::Zero();
    Mat3 _Kdw = Mat3::Zero();
    Mat3 _Kpw = Mat3::Zero();
    Mat3 _KpSwing = Mat3::Zero();
    Mat3 _KdSwing = Mat3::Zero();

};

#endif  // STATE_TROTTING_H
