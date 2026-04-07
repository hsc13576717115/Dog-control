/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"

// 被动态：不主动跟踪目标角，仅保留阻尼，作为最安全的初始状态。
class State_Passive : public FSMState{
public:
    State_Passive(CtrlComponents *ctrlComp);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
};

#endif  // PASSIVE_H
