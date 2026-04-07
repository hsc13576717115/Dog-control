#ifndef STATE_FIXEDSTAND_H
#define STATE_FIXEDSTAND_H

#include "FSMState.h"

enum StandMode {
    NORMAL_STAND = 0,
    CROUCH = 1,
};

// 固定站立状态。
// 通过 hip frame 下的目标足端位置做 IK，然后平滑插值到目标关节角。
class State_FixedStand : public FSMState {
public:
    explicit State_FixedStand(CtrlComponents* ctrlComp);
    ~State_FixedStand() override = default;

    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    Vec3 clampJointAngles(const Vec3& q) const;

    int _duration = 500;
    float _percent = 0.0f;
    Vec12 _targetPos = Vec12::Zero();
    Vec12 _startPos = Vec12::Zero();
    StandMode _currentMode = NORMAL_STAND;
};

#endif  // STATE_FIXEDSTAND_H
