/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef WIRELESSHANDLE_H
#define WIRELESSHANDLE_H

#include "message/unitree_joystick.h"  // 摇杆数据结构
#include "interface/CmdPanel.h"         // 继承CmdPanel基类
// 正确引用SDK头文件（根据你的路径：从interface到library需上两级目录）
#include "../../library/unitree_legged_sdk-3.8.0/include/unitree_legged_sdk/comm.h"

class WirelessHandle : public CmdPanel{
public:
    WirelessHandle();                  // 构造函数
    ~WirelessHandle() = default;       // 析构函数（去掉override，基类析构无virtual时无需）
    // 修正：去掉override（基类receiveHandle参数是SerialLowState*，此处参数不同）
    void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState);

private:
    xRockerBtnDataStruct _keyData;  // 存储手柄按键/摇杆数据
};

#endif  // WIRELESSHANDLE_H