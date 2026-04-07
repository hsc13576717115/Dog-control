#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>

#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "control/CtrlComponents.h"
#include "interface/CmdPanel.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"

// 所有状态的公共基类。
// 通过 _ctrlComp / _lowCmd / _lowState 访问共享上下文和当前低层状态。
class FSMState {
public:
    FSMState(CtrlComponents* ctrlComp, FSMStateName stateName, std::string stateNameString);
    virtual ~FSMState() = default;

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

    FSMStateName _stateName;
    std::string _stateNameString;

protected:
    CtrlComponents* _ctrlComp;
    UserLowlevel::LowlevelCmd* _lowCmd;
    LowlevelState* _lowState;
    UserValue _userValue;
};

#endif  // FSMSTATE_H
