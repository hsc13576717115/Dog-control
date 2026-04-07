/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "control/ControlFrame.h"

ControlFrame::ControlFrame(CtrlComponents *ctrlComp):_ctrlComp(ctrlComp){
    // 旧实现仅仅负责包一层 FSM。
    _FSMController = new FSM(_ctrlComp);
}

void ControlFrame::run(){
    _FSMController->run();
}
