#include "FSM/State_FixedStand.h"

#include <iostream>

State_FixedStand::State_FixedStand(CtrlComponents* ctrlComp)
    : FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand") {}

void State_FixedStand::enter() {
    for (int leg = 0; leg < 4; ++leg) {
        _lowCmd->setRealStanceGain(leg);
        _lowCmd->setZeroDq(leg);
        _lowCmd->setZeroTau(leg);
    }

    for (int i = 0; i < 12; ++i) {
        _startPos(i) = _lowState->motorState[i].q;
    }

    const auto& targets = (_currentMode == NORMAL_STAND)
        ? _ctrlComp->parameters.stand_targets.normal_feet_in_hip
        : _ctrlComp->parameters.stand_targets.crouch_feet_in_hip;

    for (int leg = 0; leg < 4; ++leg) {
        // 固定站立和匍匐都统一走参数文件中的足端目标，不再散落硬编码。
        Vec3 q = _ctrlComp->robotModel->inverseKinematics(targets[leg], leg, FrameType::HIP);
        q = clampJointAngles(q);
        _targetPos.segment(leg * 3, 3) = q;
    }

    _percent = 0.0f;
    _ctrlComp->setAllStance();
}

void State_FixedStand::run() {
    // 从进入状态时的实测关节角平滑插值到目标关节角，避免突变。
    _percent += 1.0f / static_cast<float>(_duration);
    if (_percent > 1.0f) {
        _percent = 1.0f;
    }

    for (int i = 0; i < 12; ++i) {
        _lowCmd->motorCmd[i].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
        _lowCmd->motorCmd[i].q = (1.0f - _percent) * _startPos(i) + _percent * _targetPos(i);
    }
}

void State_FixedStand::exit() {
    _percent = 0.0f;
    _currentMode = (_currentMode == NORMAL_STAND) ? CROUCH : NORMAL_STAND;
}

FSMStateName State_FixedStand::checkChange() {
    if (_lowState->userCmd == UserCommand::L2_B) {
        return FSMStateName::PASSIVE;
    }
    if (_lowState->userCmd == UserCommand::L2_X) {
        return FSMStateName::TROTTING;
    }
    return FSMStateName::FIXEDSTAND;
}

Vec3 State_FixedStand::clampJointAngles(const Vec3& q) const {
    // IK 结果在这里做一次机械限位保护。
    const Vec3 lower = _ctrlComp->parameters.joint_limits.lower();
    const Vec3 upper = _ctrlComp->parameters.joint_limits.upper();
    Vec3 clamped = q;
    for (int i = 0; i < 3; ++i) {
        clamped(i) = std::min(std::max(clamped(i), lower(i)), upper(i));
    }
    return clamped;
}
