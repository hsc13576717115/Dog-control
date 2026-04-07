#include "FSM/State_SwingTest.h"

// 遗留实现：当前主线不再编译 SwingTest。
// 新版已经把原本写在本文件里的 IK / FK 封装进 robotModel。

#include <iostream>
#include <iomanip>

/* =========================================================
 * 4. 调试打印
 * ========================================================= */
static void printXZ(const std::string& tag, const Vec3& v) {
    std::cout << "[SwingTest] " << tag
              << " x=" << std::fixed << std::setprecision(4) << v.x()
              << " z=" << std::fixed << std::setprecision(4) << v.z()
              << " m\n";
}

/* =========================================================
 * 5. FSM：构造
 * ========================================================= */
State_SwingTest::State_SwingTest(CtrlComponents* ctrlComp)
    : FSMState(ctrlComp, FSMStateName::SWINGTEST, "swingTest") {}

void State_SwingTest::initializeLegControl() {
    for (int leg = 0; leg < 4; ++leg) {
        _lowCmd->setZeroDq(leg);
        _lowCmd->setZeroTau(leg);
    }
}

Vec3 State_SwingTest::readTestLegQ() const {
    return Vec3(0.0, _lowState->motorState[_testLeg * 3 + 1].q, _lowState->motorState[_testLeg * 3 + 2].q);
}

void State_SwingTest::updateTargetFootInHip() {
    _userValue = _lowState->userValue;
    _targetFootInHip = _standFootInHip;
    _targetFootInHip.x() += _userValue.ly * _xRange;
    _targetFootInHip.z() += _userValue.ry * _zRange;
    _filteredFootInHip = 0.95 * _filteredFootInHip + 0.05 * _targetFootInHip;
}

Vec3 State_SwingTest::solveDesiredLegQ() const {
    return _ctrlComp->robotModel->inverseKinematics(_filteredFootInHip, _testLeg, FrameType::HIP);
}

void State_SwingTest::applyTestLegCommand(const Vec3& q_des) {
    Vec12 qCmd;
    for (int i = 0; i < 12; ++i) {
        qCmd(i) = _lowState->motorState[i].q;
    }

    // 保持原行为：SwingTest 仅控制测试腿的 thigh / calf，髋关节仍保持当前值。
    qCmd(_testLeg * 3 + 1) = q_des(1);
    qCmd(_testLeg * 3 + 2) = q_des(2);
    _lowCmd->setQ(qCmd);
}

/* =========================================================
 * 6. enter
 * ========================================================= */
void State_SwingTest::enter() {
    initializeLegControl();

    _prevQ = readTestLegQ();
    _enterQ = _prevQ;
    _standQ = _prevQ;

    // 原本写在 SwingTest 里的 fkCheckAxis 已经封装到 robotModel 中。
    _standFootInHip = _ctrlComp->robotModel->forwardKinematics(_standQ, _testLeg, FrameType::HIP);
    _targetFootInHip = _standFootInHip;
    _filteredFootInHip = _standFootInHip;

    _transitionPct = 0.0f;

    printXZ("Enter Foot", _standFootInHip);
    _ctrlComp->setAllSwing();
}

/* =========================================================
 * 7. run
 * ========================================================= */
void State_SwingTest::run() {
    updateTargetFootInHip();

    // 原本写在 SwingTest 里的 ikCheckAxis 已经封装到 robotModel 中。
    Vec3 qDes = solveDesiredLegQ();

    if (_transitionPct < 1.0f) {
        _transitionPct += 1.0f / _transitionDuration;
        qDes = _prevQ + (_enterQ - _prevQ) * _transitionPct;
    }

    applyTestLegCommand(qDes);
}

/* =========================================================
 * 8. exit / checkChange
 * ========================================================= */
void State_SwingTest::exit() {
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_SwingTest::checkChange() {
    if (_lowState->userCmd == UserCommand::L2_B)
        return FSMStateName::PASSIVE;
    if (_lowState->userCmd == UserCommand::L2_A)
        return FSMStateName::FIXEDSTAND;
    return FSMStateName::SWINGTEST;
}
