#include "FSM/State_Passive.h"
#include "FSM/StateMotorParams.h"

State_Passive::State_Passive(CtrlComponents* ctrlComp)
    : FSMState(ctrlComp, FSMStateName::PASSIVE, "passive") {}

void State_Passive::enter() {
    // passive 状态下不施加位置环，仅保留一定阻尼帮助站立时更平顺。
    for (int leg = 0; leg < 4; ++leg) {
        fsm_motor_params::ApplyLegProfile(_lowCmd, leg, fsm_motor_params::kPassiveProfile);
        for (int joint = 0; joint < 3; ++joint) {
            _lowCmd->motorCmd[leg * 3 + joint].q = 0.0f;
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
