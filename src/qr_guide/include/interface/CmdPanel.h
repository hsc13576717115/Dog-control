#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "message/unitree_joystick.h"
#include "common/enumClass.h"
#include <cstdio>
#include <pthread.h>
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"
// 引入用户命令头文件
#include "message/LowlevelCmd.h"

// 串口链路下的简化 low state，仅用于兼容旧接口中的接收回调。
struct SerialLowState {
    MotorData motorData[12];  // SDK的MotorData（全局）
    
    SerialLowState() {
        for (int i = 0; i < 12; i++) {
            motorData[i].motorType = MotorType::GO_M8010_6;  // 全局枚举
        }
    }
};

// 连续手柄输入值。
struct UserValue{
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    UserValue(){ setZero(); }
    void setZero(){ lx=0; ly=0; rx=0; ry=0; L2=0; }
};

// 命令面板基类。
// 当前主线更多是把它作为 UserCommand / UserValue 的承载结构，而不是线程实体。
class CmdPanel{
public:
    CmdPanel(){}
    virtual ~CmdPanel(){}
    UserCommand getUserCmd(){return userCmd;}
    UserValue getUserValue(){return userValue;}
    void setPassive(){userCmd = UserCommand::L2_B;}
    void setZero(){userValue.setZero();}

#ifdef COMPILE_WITH_REAL_ROBOT
    virtual void receiveHandle(SerialLowState *serialState) {
        for (int i = 0; i < 12; i++) {
            if (serialState->motorData[i].temp > 70) {
                // 简单的过温提示，真正的保护策略仍然建议在更高层完成。
                printf("Motor %d overheat! Temp: %d℃\n", i, serialState->motorData[i].temp);
            }
        }
    }
#endif

protected:
    virtual void* run(void *arg){return NULL;}
    UserCommand userCmd;
    UserValue userValue;
};

#endif  // CMDPANEL_H
