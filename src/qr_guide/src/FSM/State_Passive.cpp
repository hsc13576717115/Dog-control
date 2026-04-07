#include "FSM/State_Passive.h"

State_Passive::State_Passive(CtrlComponents* ctrlComp)
    : FSMState(ctrlComp, FSMStateName::PASSIVE, "passive") {}

void State_Passive::enter() {
    constexpr float kHipKd = 300.0f;
    constexpr float kThighKd = 3.0f;
    constexpr float kCalfKd = 3.0f;

    // passive 状态下不施加位置环，仅保留一定阻尼帮助站立时更平顺。
    for (int i = 0; i < 12; ++i) {
        _lowCmd->motorCmd[i].mode = 0;
        _lowCmd->motorCmd[i].q = 0.0f;
        _lowCmd->motorCmd[i].dq = 0.0f;
        _lowCmd->motorCmd[i].Kp = 0.0f;
        _lowCmd->motorCmd[i].tau = 0.0f;

        if (i == 0 || i == 3 || i == 6 || i == 9) {
            _lowCmd->motorCmd[i].Kd = kHipKd;
        } else if (i == 1 || i == 4 || i == 7 || i == 10) {
            _lowCmd->motorCmd[i].Kd = kThighKd;
        } else {
            _lowCmd->motorCmd[i].Kd = kCalfKd;
        }
    }

    _ctrlComp->setAllSwing();
}

void State_Passive::run() {}

void State_Passive::exit() {}

FSMStateName State_Passive::checkChange() {
    // 只允许由 L2_A 进入固定站立，避免从 passive 直接切到更激进的状态。
    if (_lowState->userCmd == UserCommand::L2_A) {
        return FSMStateName::FIXEDSTAND;
    }
    return FSMStateName::PASSIVE;
}
