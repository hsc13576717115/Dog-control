#include "FSM/FSMState.h"

FSMState::FSMState(CtrlComponents* ctrlComp, FSMStateName stateName, std::string stateNameString)
    : _stateName(stateName),
      _stateNameString(std::move(stateNameString)),
      _ctrlComp(ctrlComp),
      // 这里缓存底层指针只是为了减少状态实现里的重复访问。
      _lowCmd(_ctrlComp->lowCmd.get()),
      _lowState(_ctrlComp->lowState.get()) {}
