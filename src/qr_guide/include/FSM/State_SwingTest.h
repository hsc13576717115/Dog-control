#ifndef STATE_SWINGTEST_H
#define STATE_SWINGTEST_H

#include "FSMState.h"

// 遗留状态：早期的摆腿测试实现。
// 当前主构建不再编译该状态，但这里仍保留作单腿调试参考。
// 原本写在本文件里的 fkCheckAxis / ikCheckAxis 已经封装到 robotModel 中。

class State_SwingTest : public FSMState {
public:
    State_SwingTest(CtrlComponents* ctrlComp);
    ~State_SwingTest() = default;

    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    void initializeLegControl();
    Vec3 readTestLegQ() const;
    void updateTargetFootInHip();
    Vec3 solveDesiredLegQ() const;
    void applyTestLegCommand(const Vec3& q_des);

    static constexpr int _transitionDuration = 50;
    static constexpr int _testLeg = 0;
    static constexpr double _xRange = 0.020;
    static constexpr double _zRange = 0.020;

    float _transitionPct = 0.0f;
    Vec3 _standQ = Vec3::Zero();
    Vec3 _enterQ = Vec3::Zero();
    Vec3 _prevQ = Vec3::Zero();
    Vec3 _standFootInHip = Vec3::Zero();
    Vec3 _targetFootInHip = Vec3::Zero();
    Vec3 _filteredFootInHip = Vec3::Zero();
};

#endif  // STATE_SWINGTEST_H
