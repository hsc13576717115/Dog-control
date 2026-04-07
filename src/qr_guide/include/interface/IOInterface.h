// /**********************************************************************
//  Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
// ***********************************************************************/
// #ifndef IOINTERFACE_H
// #define IOINTERFACE_H

// #include "message/LowlevelCmd.h"
// #include "message/LowlevelState.h"
// #include "interface/CmdPanel.h"
// #include <string>

// class IOInterface{
// public:
// IOInterface(){}
// ~IOInterface(){delete cmdPanel;}
// virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
// void zeroCmdPanel(){cmdPanel->setZero();}
// void setPassive(){cmdPanel->setPassive();}

// protected:
// CmdPanel *cmdPanel;
// };

// #endif  //IOINTERFACE_H

/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "interface/CmdPanel.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"

class IOInterface {
public:
    IOInterface() : cmdPanel(new CmdPanel()) {}
    virtual ~IOInterface() {
        delete cmdPanel;
        cmdPanel = nullptr;
    }

    // 硬件层统一接口：发命令并同步刷新 LowlevelState。
    virtual void sendRecv(const UserLowlevel::LowlevelCmd* cmd, LowlevelState* state) = 0;
    // 默认返回 false，真实硬件实现可覆写成自己的校准状态。
    virtual bool isCalibrated() const { return false; }

    void zeroCmdPanel() { cmdPanel->setZero(); }
    void setPassive() { cmdPanel->setPassive(); }

protected:
    CmdPanel* cmdPanel;
};

#endif  // IOINTERFACE_H
