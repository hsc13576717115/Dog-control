// /**********************************************************************
//  Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
// ***********************************************************************/
// #include "FSM/State_Passive.h"

// State_Passive::State_Passive(CtrlComponents *ctrlComp)
//              :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){}

// void State_Passive::enter(){
//     if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
//         for(int i=0; i<12; i++){
//             _lowCmd->motorCmd[i].mode = 0;
//             _lowCmd->motorCmd[i].q = 0;
//             _lowCmd->motorCmd[i].dq = 0;
//             _lowCmd->motorCmd[i].Kp = 0;
//             _lowCmd->motorCmd[i].Kd = 8;
//             _lowCmd->motorCmd[i].tau = 0;
//         }
//     }
//     else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
//         for(int i=0; i<12; i++){
//             _lowCmd->motorCmd[i].mode = 0;
//             _lowCmd->motorCmd[i].q = 0;
//             _lowCmd->motorCmd[i].dq = 0;
//             _lowCmd->motorCmd[i].Kp = 0;
//             _lowCmd->motorCmd[i].Kd = 8;
//             _lowCmd->motorCmd[i].tau = 0;
//         }
//     }

//     _ctrlComp->setAllSwing();
// }

// void State_Passive::run(){
    
// }

// void State_Passive::exit(){

// }

// FSMStateName State_Passive::checkChange(){
//     if(_lowState->userCmd == UserCommand::L2_A){
//         std::cout << "[DEBUG] 检测到L2+A指令，切换到站立模式" << std::endl;
//         return FSMStateName::FIXEDSTAND;
//     }
//     else{
//         return FSMStateName::PASSIVE;
//     }
// }

#include "FSM/State_Passive.h"

State_Passive::State_Passive(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){}

void State_Passive::enter(){
    // 1. 定义不同关节的Kd值（可根据需求修改，示例值仅供参考）
    const float HIP_KD = 300.0f;    // 髋关节电机Kd
    const float THIGH_KD = 3.0f;  // 大腿（膝关节）电机Kd
    const float CALF_KD = 3.0f;  // 小腿（踝关节）电机Kd

    if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        for(int i=0; i<12; i++){  // i是电机ID（0-11）
            _lowCmd->motorCmd[i].mode = 0;
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;
            
            // 2. 按电机ID判断关节类型，赋值对应Kd
            if(i == 0 || i == 3 || i == 6 || i == 9){  // 所有髋关节电机ID
                _lowCmd->motorCmd[i].Kd = HIP_KD;
            }
            else if(i == 1 || i == 4 || i == 7 || i == 10){  // 所有大腿电机ID
                _lowCmd->motorCmd[i].Kd = THIGH_KD;
            }
            else{  // 剩余为小腿电机ID（2,5,8,11）
                _lowCmd->motorCmd[i].Kd = CALF_KD;
            }

            _lowCmd->motorCmd[i].tau = 0;
        }
    }
    else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
        for(int i=0; i<12; i++){  // 同理，REALROBOT平台也按关节类型赋值Kd
            _lowCmd->motorCmd[i].mode = 0;
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;
            
            // 3. 与GAZEBO平台逻辑一致，区分关节类型
            if(i == 0 || i == 3 || i == 6 || i == 9){  // 髋关节
                _lowCmd->motorCmd[i].Kd = HIP_KD;
            }
            else if(i == 1 || i == 4 || i == 7 || i == 10){  // 大腿
                _lowCmd->motorCmd[i].Kd = THIGH_KD;
            }
            else{  // 小腿
                _lowCmd->motorCmd[i].Kd = CALF_KD;
            }

            _lowCmd->motorCmd[i].tau = 0;
        }
    }

    _ctrlComp->setAllSwing();
}

// 以下run()、exit()、checkChange()函数保持不变
void State_Passive::run(){
    
}

void State_Passive::exit(){

}

FSMStateName State_Passive::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_A){
        std::cout << "[DEBUG] 检测到L2+A指令，切换到站立模式" << std::endl;
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}