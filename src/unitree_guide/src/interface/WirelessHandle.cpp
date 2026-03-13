/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_REAL_ROBOT  // 仅在实物机器人模式下编译

#include "interface/WirelessHandle.h"
#include "common/mathTools.h"    // 包含killZeroOffset函数
#include <string.h>              // 用于memcpy
#include <stdio.h>               // 用于调试输出

// 构造函数：初始化手柄数据
WirelessHandle::WirelessHandle() {
    // 初始化按键数据为0
    memset(&_keyData, 0, sizeof(xRockerBtnDataStruct));
    printf("[WirelessHandle] 初始化成功（SDK路径：library/unitree_legged_sdk-3.8.0）\n");
}

// 接收并解析手柄数据（从LowState中提取）
void WirelessHandle::receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState) {
    if (lowState == nullptr) {
        printf("[WirelessHandle] 错误：lowState为空指针\n");
        return;
    }

    // 根据机器人类型解析无线手柄数据（A1/Go1的存储格式不同）
#ifdef ROBOT_TYPE_A1
    // A1机器人：wirelessRemote是数组，直接 memcpy
    memcpy(&_keyData, lowState->wirelessRemote, sizeof(xRockerBtnDataStruct));
#endif
#ifdef ROBOT_TYPE_Go1
    // Go1机器人：wirelessRemote是向量，取首地址
    memcpy(&_keyData, &lowState->wirelessRemote[0], sizeof(xRockerBtnDataStruct));
#endif

    // 解析组合按键命令（优先级从高到低）
    userCmd = UserCommand::NONE;  // 默认无命令

    if (((int)_keyData.btn.components.L2 == 1) && ((int)_keyData.btn.components.B == 1)) {
        userCmd = UserCommand::L2_B;  // L2+B：被动模式
    } else if (((int)_keyData.btn.components.L2 == 1) && ((int)_keyData.btn.components.A == 1)) {
        userCmd = UserCommand::L2_A;  // L2+A：站立模式
    } else if (((int)_keyData.btn.components.L2 == 1) && ((int)_keyData.btn.components.X == 1)) {
        userCmd = UserCommand::L2_X;  // L2+X：行走模式
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if (((int)_keyData.btn.components.L2 == 1) && ((int)_keyData.btn.components.Y == 1)) {
        userCmd = UserCommand::L2_Y;  // L2+Y：移动底盘模式（仅在move_base编译时生效）
    }
#endif  // COMPILE_WITH_MOVE_BASE
    else if (((int)_keyData.btn.components.L1 == 1) && ((int)_keyData.btn.components.X == 1)) {
        userCmd = UserCommand::L1_X;  // L1+X：自定义功能1
    } else if (((int)_keyData.btn.components.L1 == 1) && ((int)_keyData.btn.components.A == 1)) {
        userCmd = UserCommand::L1_A;  // L1+A：自定义功能2
    } else if (((int)_keyData.btn.components.L1 == 1) && ((int)_keyData.btn.components.Y == 1)) {
        userCmd = UserCommand::L1_Y;  // L1+Y：自定义功能3
    } else if ((int)_keyData.btn.components.start == 1) {
        userCmd = UserCommand::START;  // START键：紧急停止/复位
    }

    // 解析摇杆值（消除零位偏移，小于0.08的值视为0）
    userValue.L2 = killZeroOffset(_keyData.L2, 0.08);  // L2扳机键（0~1）
    userValue.lx = killZeroOffset(_keyData.lx, 0.08);  // 左摇杆X轴（-1~1）
    userValue.ly = killZeroOffset(_keyData.ly, 0.08);  // 左摇杆Y轴（-1~1）
    userValue.rx = killZeroOffset(_keyData.rx, 0.08);  // 右摇杆X轴（-1~1）
    userValue.ry = killZeroOffset(_keyData.ry, 0.08);  // 右摇杆Y轴（-1~1）

    // 调试输出（每100次调用打印一次，避免刷屏）
    static int count = 0;
    if (++count % 100 == 0) {
        printf("[WirelessHandle] 摇杆值：L2=%.2f, lx=%.2f, ly=%.2f, rx=%.2f, ry=%.2f\n",
               userValue.L2, userValue.lx, userValue.ly, userValue.rx, userValue.ry);
    }
}

#endif  // COMPILE_WITH_REAL_ROBOT
