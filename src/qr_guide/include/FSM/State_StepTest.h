#ifndef STATE_STEPTEST_H
#define STATE_STEPTEST_H

#include <string>

#include "FSM/FSMState.h"

// StepTest 当前主要承载跳跃/大步动作验证。
// 轨迹仍然采用基于时间片段的足端目标 + IK 输出。
class State_StepTest : public FSMState {
public:
    explicit State_StepTest(CtrlComponents* ctrlComp);
    ~State_StepTest() override = default;

    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    Vec3 clampJointAngles(const Vec3& q) const;

    int _transitionCount = 0;
    Vec12 _initMotorQ = Vec12::Zero();
    Vec3 _initFootPos[4];
    Vec3 _lastLegQ[4];
    double _startTime = 0.0;
    bool _isJumpCompleted = false;
    bool _isCycleEnded = false;

    static constexpr double SQUAT_DEPTH = 0.15;
    static constexpr double JUMP_HEIGHT = 0.14;
    static constexpr double MAX_X_FORWARD = 0.30;
    static constexpr double LANDING_OFFSET = 0.08;
    static constexpr double JUMP_CYCLE_T = 3.5;
    static constexpr int TRANSITION_DURATION = 50;
    static constexpr double HIP_JOINT_FIXED = 0.0;
};

#endif  // STATE_STEPTEST_H
