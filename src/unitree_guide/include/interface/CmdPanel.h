// // /**********************************************************************
// //  Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
// // ***********************************************************************/
// // #ifndef CMDPANEL_H
// // #define CMDPANEL_H

// // #include "message/unitree_joystick.h"
// // #include "common/enumClass.h"
// // #include <pthread.h>

// // #ifdef COMPILE_WITH_REAL_ROBOT
// //     #ifdef ROBOT_TYPE_A1
// //         #include "unitree_legged_sdk/unitree_legged_sdk.h"
// //     #endif  // ROBOT_TYPE_A1
// //     #ifdef ROBOT_TYPE_Go1
// //         #include "unitree_legged_sdk/unitree_legged_sdk.h"
// //     #endif  // ROBOT_TYPE_Go1
// // #endif  // COMPILE_WITH_REAL_ROBOT

// // struct UserValue{
// //     float lx;
// //     float ly;
// //     float rx;
// //     float ry;
// //     float L2;
// //     UserValue(){
// //         setZero();
// //     }
// //     void setZero(){
// //         lx = 0;
// //         ly = 0;
// //         rx = 0;
// //         ry = 0;
// //         L2 = 0;
// //     }
// // };

// // class CmdPanel{
// // public:
// //     CmdPanel(){}
// //     virtual ~CmdPanel(){}
// //     UserCommand getUserCmd(){return userCmd;}
// //     UserValue getUserValue(){return userValue;}
// //     void setPassive(){userCmd = UserCommand::L2_B;}
// //     void setZero(){userValue.setZero();}
// // #ifdef COMPILE_WITH_REAL_ROBOT
// //     virtual void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){};
// // #endif  // COMPILE_WITH_REAL_ROBOT
// // protected:
// //     virtual void* run(void *arg){return NULL;}
// //     UserCommand userCmd;
// //     UserValue userValue;
// // };

// // #endif  // CMDPANEL_H


#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "message/unitree_joystick.h"
#include "common/enumClass.h"
#include <pthread.h>
#include "thirdParty/unitree_actuator_sdk-main/include/unitreeMotor/unitreeMotor.h"
#include "thirdParty/unitree_actuator_sdk-main/include/serialPort/SerialPort.h"
// 引入用户命令头文件
#include "message/LowlevelCmd.h"

// 自定义状态结构（使用SDK的全局MotorData）
struct SerialLowState {
    MotorData motorData[12];  // SDK的MotorData（全局）
    
    SerialLowState() {
        for (int i = 0; i < 12; i++) {
            motorData[i].motorType = MotorType::GO_M8010_6;  // 全局枚举
        }
    }
};

struct UserValue{
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    UserValue(){ setZero(); }
    void setZero(){ lx=0; ly=0; rx=0; ry=0; L2=0; }
};

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
                // 修正printf格式：int类型使用%d
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
