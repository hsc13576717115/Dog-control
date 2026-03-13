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

#include "message/LowlevelCmd.h"  // 包含UserLowlevel命名空间
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>

class IOInterface{
public:
    // 构造函数：初始化cmdPanel，避免空指针
    IOInterface() : cmdPanel(new CmdPanel()) {}
    
    // 析构函数：确保释放资源
    ~IOInterface(){
        if(cmdPanel != nullptr){
            delete cmdPanel;
            cmdPanel = nullptr;
        }
    }
    
    // 核心修改：使用UserLowlevel命名空间的LowlevelCmd
    virtual void sendRecv(const UserLowlevel::LowlevelCmd *cmd, LowlevelState *state) = 0;
    
    void zeroCmdPanel(){cmdPanel->setZero();}
    void setPassive(){cmdPanel->setPassive();}

protected:
    CmdPanel *cmdPanel;  // 由构造函数初始化，避免空指针
};

#endif  //IOINTERFACE_H