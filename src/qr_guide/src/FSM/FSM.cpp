#include "FSM/FSM.h"

#include <iostream>

FSM::FSM(CtrlComponents* ctrlComp)
    : _ctrlComp(ctrlComp) {
    // 主线只实例化当前保留的 4 个状态。
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
    initialize();
}

FSM::~FSM() {
    _stateList.deletePtr();
}

void FSM::initialize() {
    // 系统默认从 passive 启动，避免上电直接进入有力矩状态。
    _currentState = _stateList.passive;
    _nextState = _currentState;
    _currentState->enter();
    _mode = FSMMode::NORMAL;
}

void FSM::run() {
    if (_mode == FSMMode::NORMAL) {
        // 正常模式：先运行当前状态，再检查是否需要切换。
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if (_nextStateName != _currentState->_stateName) {
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
        return;
    }

    _currentState->exit();
    // 切换模式：先退出旧状态，再进入新状态。
    _currentState = _nextState;
    _currentState->enter();
    _mode = FSMMode::NORMAL;
    _currentState->run();
}

FSMState* FSM::getNextState(FSMStateName stateName) {
    switch (stateName) {
        case FSMStateName::PASSIVE:
            return _stateList.passive;
        case FSMStateName::FIXEDSTAND:
            return _stateList.fixedStand;
        case FSMStateName::TROTTING:
            return _stateList.trotting;
        case FSMStateName::STEPTEST:
            return _stateList.stepTest;
        default:
            return _stateList.passive;
    }
}
