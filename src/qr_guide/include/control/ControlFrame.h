/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CONTROLFRAME_H
#define CONTROLFRAME_H

#include "FSM/FSM.h"
#include "control/CtrlComponents.h"

// 旧版控制壳层，当前主线已经由 RobotRunner 取代。
// 这里保留只是为了兼容历史代码引用。
class ControlFrame{
public:
	ControlFrame(CtrlComponents *ctrlComp);
	~ControlFrame(){
		delete _FSMController;
	}
	void run();
private:
	FSM* _FSMController;
	CtrlComponents *_ctrlComp;
};

#endif  //CONTROLFRAME_H
