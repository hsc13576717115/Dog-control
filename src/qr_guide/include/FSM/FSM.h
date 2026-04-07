#ifndef FSM_H
#define FSM_H

#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_StepTest.h"
#include "FSM/State_Trotting.h"

// 主线状态列表。
// 当前只保留 Passive / FixedStand / Trotting / StepTest 四个状态参与编译。
struct FSMStateList {
    State_Passive* passive = nullptr;
    State_FixedStand* fixedStand = nullptr;
    State_Trotting* trotting = nullptr;
    State_StepTest* stepTest = nullptr;

    void deletePtr() {
        delete passive;
        delete fixedStand;
        delete trotting;
        delete stepTest;
    }
};

// 主状态机。
// 只负责当前状态运行和状态切换，不再直接处理 IO、估计器和 ROS 回调。
class FSM {
public:
    explicit FSM(CtrlComponents* ctrlComp);
    ~FSM();

    void initialize();
    void run();
    FSMStateName currentStateName() const { return _currentState->_stateName; }
    std::string currentStateLabel() const {
        return _currentState ? _currentState->_stateNameString : "unknown";
    }

private:
    FSMState* getNextState(FSMStateName stateName);

    CtrlComponents* _ctrlComp;
    FSMState* _currentState = nullptr;
    FSMState* _nextState = nullptr;
    FSMStateName _nextStateName = FSMStateName::INVALID;
    FSMStateList _stateList;
    FSMMode _mode = FSMMode::NORMAL;
};

#endif  // FSM_H
