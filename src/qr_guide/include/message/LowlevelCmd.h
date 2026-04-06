// /**********************************************************************
//  Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
// ***********************************************************************/
// #ifndef LOWLEVELCMD_H
// #define LOWLEVELCMD_H

// #include "common/mathTypes.h"
// #include "common/mathTools.h"

// struct MotorCmd{
//     unsigned int mode;
//     float q;
//     float dq;
//     float tau;
//     float Kp;
//     float Kd;

//     MotorCmd(){
//         mode = 0;
//         q = 0;
//         dq = 0;
//         tau = 0;
//         Kp = 0;
//         Kd = 0;
//     }
// };

// struct LowlevelCmd{
//     MotorCmd motorCmd[12];

//     void setQ(Vec12 q){
//         for(int i(0); i<12; ++i){
//             motorCmd[i].q = q(i);
//         }
//     }
//     void setQ(int legID, Vec3 qi){
//         motorCmd[legID*3+0].q = qi(0);
//         motorCmd[legID*3+1].q = qi(1);
//         motorCmd[legID*3+2].q = qi(2);
//     }
//     void setQd(Vec12 qd){
//         for(int i(0); i<12; ++i){
//             motorCmd[i].dq = qd(i);
//         }
//     }
//     void setQd(int legID, Vec3 qdi){
//         motorCmd[legID*3+0].dq = qdi(0);
//         motorCmd[legID*3+1].dq = qdi(1);
//         motorCmd[legID*3+2].dq = qdi(2);
//     }
//     void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-50, 50)){
//         for(int i(0); i<12; ++i){
//             if(std::isnan(tau(i))){
//                 printf("[ERROR] The setTau function meets Nan\n");
//             }
//             motorCmd[i].tau = saturation(tau(i), torqueLimit);
//         }
//     }
//     void setZeroDq(int legID){
//         motorCmd[legID*3+0].dq = 0;
//         motorCmd[legID*3+1].dq = 0;
//         motorCmd[legID*3+2].dq = 0;
//     }
//     void setZeroDq(){
//         for(int i(0); i<4; ++i){
//             setZeroDq(i);
//         }
//     }
//     void setZeroTau(int legID){
//         motorCmd[legID*3+0].tau = 0;
//         motorCmd[legID*3+1].tau = 0;
//         motorCmd[legID*3+2].tau = 0;
//     }
//     void setSimStanceGain(int legID){
//         motorCmd[legID*3+0].mode = 10;
//         motorCmd[legID*3+0].Kp = 180;
//         motorCmd[legID*3+0].Kd = 8;
//         motorCmd[legID*3+1].mode = 10;
//         motorCmd[legID*3+1].Kp = 180;
//         motorCmd[legID*3+1].Kd = 8;
//         motorCmd[legID*3+2].mode = 10;
//         motorCmd[legID*3+2].Kp = 300;
//         motorCmd[legID*3+2].Kd = 15;
//     }
//     void setRealStanceGain(int legID){
//         motorCmd[legID*3+0].mode = 10;
//         motorCmd[legID*3+0].Kp = 60;
//         motorCmd[legID*3+0].Kd = 5;
//         motorCmd[legID*3+1].mode = 10;
//         motorCmd[legID*3+1].Kp = 40;
//         motorCmd[legID*3+1].Kd = 4;
//         motorCmd[legID*3+2].mode = 10;
//         motorCmd[legID*3+2].Kp = 80;
//         motorCmd[legID*3+2].Kd = 7;
//     }
//     void setZeroGain(int legID){
//         motorCmd[legID*3+0].mode = 10;
//         motorCmd[legID*3+0].Kp = 0;
//         motorCmd[legID*3+0].Kd = 0;
//         motorCmd[legID*3+1].mode = 10;
//         motorCmd[legID*3+1].Kp = 0;
//         motorCmd[legID*3+1].Kd = 0;
//         motorCmd[legID*3+2].mode = 10;
//         motorCmd[legID*3+2].Kp = 0;
//         motorCmd[legID*3+2].Kd = 0;
//     }
//     void setZeroGain(){
//         for(int i(0); i<4; ++i){
//             setZeroGain(i);
//         }
//     }
//     void setStableGain(int legID){
//         motorCmd[legID*3+0].mode = 10;
//         motorCmd[legID*3+0].Kp = 0.8;
//         motorCmd[legID*3+0].Kd = 0.8;
//         motorCmd[legID*3+1].mode = 10;
//         motorCmd[legID*3+1].Kp = 0.8;
//         motorCmd[legID*3+1].Kd = 0.8;
//         motorCmd[legID*3+2].mode = 10;
//         motorCmd[legID*3+2].Kp = 0.8;
//         motorCmd[legID*3+2].Kd = 0.8;
//     }
//     void setStableGain(){
//         for(int i(0); i<4; ++i){
//             setStableGain(i);
//         }
//     }
//     void setSwingGain(int legID){
//         motorCmd[legID*3+0].mode = 10;
//         motorCmd[legID*3+0].Kp = 3;
//         motorCmd[legID*3+0].Kd = 2;
//         motorCmd[legID*3+1].mode = 10;
//         motorCmd[legID*3+1].Kp = 3;
//         motorCmd[legID*3+1].Kd = 2;
//         motorCmd[legID*3+2].mode = 10;
//         motorCmd[legID*3+2].Kp = 3;
//         motorCmd[legID*3+2].Kd = 2;
//     }
// };

// #endif  //LOWLEVELCMD_H

/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/mathTypes.h"
#include "common/mathTools.h"

enum class ControlMode {
    DISABLE = 0,       // 禁用
    COMPOUND = 1      // 位置+速度+力矩复合控制模式（GO-M8010-6默认）
};

namespace UserLowlevel {
    /**
     * @brief 单电机控制命令结构
     * 包含模式、目标位置、速度、力矩及PID增益
     */
    struct MotorCmd {
        unsigned int mode;  // 控制模式（参考ControlMode枚举）
        float q;            // 目标位置（弧度，输出侧）
        float dq;           // 目标速度（弧度/秒，输出侧）
        float tau;          // 目标力矩（N·m）
        float Kp;           // 位置比例增益
        float Kd;           // 速度比例增益

        // 构造函数：使用初始化列表（比赋值更高效）
        MotorCmd() 
            : mode(static_cast<unsigned int>(ControlMode::DISABLE)), 
              q(0.0f), dq(0.0f), tau(0.0f), Kp(0.0f), Kd(0.0f) {}
    };

    /**
     * @brief 12电机整体控制命令结构
     * 提供批量设置位置、速度、力矩及增益的接口
     */
    struct LowlevelCmd {
        MotorCmd motorCmd[12];  // 12个电机的命令（索引0-11对应12个关节）

        /**
         * @brief 设置所有电机的目标位置
         * @param q 12维向量，依次对应12个电机的目标位置（弧度）
         */
        void setQ(const Vec12& q) {  // 加const&避免拷贝
            for (int i = 0; i < 12; ++i) {
                motorCmd[i].q = q(i);
            }
        }

        /**
         * @brief 设置单条腿的3个电机目标位置
         * @param legID 腿索引（0:FR, 1:FL, 2:RR, 3:RL）
         * @param qi 3维向量，对应腿内髋关节、大腿、小腿电机位置
         */
        void setQ(int legID, const Vec3& qi) {  // 加const&
            if (legID < 0 || legID >= 4) {  // 索引检查，避免越界
                printf("[LowlevelCmd] 错误：legID=%d（应在0-3之间）\n", legID);
                return;
            }
            motorCmd[legID*3 + 0].q = qi(0);  // 髋关节
            motorCmd[legID*3 + 1].q = qi(1);  // 大腿电机
            motorCmd[legID*3 + 2].q = qi(2);  // 小腿电机
        }

        /**
         * @brief 设置所有电机的目标速度
         * @param qd 12维向量，依次对应12个电机的目标速度（弧度/秒）
         */
        void setQd(const Vec12& qd) {  // 加const&
            for (int i = 0; i < 12; ++i) {
                motorCmd[i].dq = qd(i);
            }
        }

        /**
         * @brief 设置单条腿的3个电机目标速度
         * @param legID 腿索引（0:FR, 1:FL, 2:RR, 3:RL）
         * @param qdi 3维向量，对应腿内3个电机的目标速度
         */
        void setQd(int legID, const Vec3& qdi) {  // 加const&
            if (legID < 0 || legID >= 4) {
                printf("[LowlevelCmd] 错误：legID=%d（应在0-3之间）\n", legID);
                return;
            }
            motorCmd[legID*3 + 0].dq = qdi(0);
            motorCmd[legID*3 + 1].dq = qdi(1);
            motorCmd[legID*3 + 2].dq = qdi(2);
        }

        /**
         * @brief 设置所有电机的目标力矩（带饱和限制）
         * @param tau 12维向量，对应12个电机的目标力矩
         * @param torqueLimit 力矩上下限（默认±50N·m）
         */
        void setTau(const Vec12& tau, const Vec2& torqueLimit = Vec2(-50, 50)) {  // 加const&
            for (int i = 0; i < 12; ++i) {
                if (std::isnan(tau(i))) {
                    printf("[LowlevelCmd] 错误：电机%d的力矩为NaN\n", i);
                }
                motorCmd[i].tau = saturation(tau(i), torqueLimit);  // 限幅保护
            }
        }

        /**
         * @brief 清零单条腿的目标速度
         * @param legID 腿索引（0-3）
         */
        void setZeroDq(int legID) {
            if (legID < 0 || legID >= 4) {
                printf("[LowlevelCmd] 错误：legID=%d（应在0-3之间）\n", legID);
                return;
            }
            motorCmd[legID*3 + 0].dq = 0.0f;
            motorCmd[legID*3 + 1].dq = 0.0f;
            motorCmd[legID*3 + 2].dq = 0.0f;
        }

        /**
         * @brief 清零所有电机的目标速度
         */
        void setZeroDq() {
            for (int i = 0; i < 4; ++i) {  // 遍历4条腿
                setZeroDq(i);
            }
        }

        /**
         * @brief 清零单条腿的目标力矩
         * @param legID 腿索引（0-3）
         */
        void setZeroTau(int legID) {
            if (legID < 0 || legID >= 4) {
                printf("[LowlevelCmd] 错误：legID=%d（应在0-3之间）\n", legID);
                return;
            }
            motorCmd[legID*3 + 0].tau = 0.0f;
            motorCmd[legID*3 + 1].tau = 0.0f;
            motorCmd[legID*3 + 2].tau = 0.0f;
        }

        /**
         * @brief 设置单条腿的仿真模式 stance 阶段增益（Gazebo用）
         * @param legID 腿索引（0-3）
         */
        void setSimStanceGain(int legID) {
            if (legID < 0 || legID >= 4) {
                printf("[LowlevelCmd] 错误：legID=%d（应在0-3之间）\n", legID);
                return;
            }
            motorCmd[legID*3 + 0].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 0].Kp = 180.0f;
            motorCmd[legID*3 + 0].Kd = 8.0f;
            motorCmd[legID*3 + 1].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 1].Kp = 180.0f;
            motorCmd[legID*3 + 1].Kd = 8.0f;
            motorCmd[legID*3 + 2].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 2].Kp = 300.0f;
            motorCmd[legID*3 + 2].Kd = 15.0f;
        }

        /**
         * @brief 设置单条腿的实物模式 stance 阶段增益（实际机器人用）
         * @param legID 腿索引（0-3）
         */
        void setRealStanceGain(int legID) {
            if (legID < 0 || legID >= 4) {
                printf("[LowlevelCmd] 错误：legID=%d（应在0-3之间）\n", legID);
                return;
            }
            motorCmd[legID*3 + 0].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 0].Kp = 6.2f;
            motorCmd[legID*3 + 0].Kd = 0.2f;
            motorCmd[legID*3 + 1].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 1].Kp = 6.2f;
            motorCmd[legID*3 + 1].Kd = 0.2f;
            motorCmd[legID*3 + 2].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 2].Kp = 6.2f;
            motorCmd[legID*3 + 2].Kd = 0.2f;
        }

        /**
         * @brief 清零单条腿的PID增益（无位置/速度控制）
         * @param legID 腿索引（0-3）
         */
        void setZeroGain(int legID) {
            if (legID < 0 || legID >= 4) {
                printf("[LowlevelCmd] 错误：legID=%d（应在0-3之间）\n", legID);
                return;
            }
            motorCmd[legID*3 + 0].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 0].Kp = 0.0f;
            motorCmd[legID*3 + 0].Kd = 0.0f;
            motorCmd[legID*3 + 1].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 1].Kp = 0.0f;
            motorCmd[legID*3 + 1].Kd = 0.0f;
            motorCmd[legID*3 + 2].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 2].Kp = 0.0f;
            motorCmd[legID*3 + 2].Kd = 0.0f;
        }

        /**
         * @brief 清零所有电机的PID增益
         */
        void setZeroGain() {
            for (int i = 0; i < 4; ++i) {
                setZeroGain(i);
            }
        }

        /**
         * @brief 设置单条腿的稳定模式增益
         * @param legID 腿索引（0-3）
         */
        void setStableGain(int legID) {
            if (legID < 0 || legID >= 4) {
                printf("[LowlevelCmd] 错误：legID=%d（应在0-3之间）\n", legID);
                return;
            }
            motorCmd[legID*3 + 0].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 0].Kp = 0.8f;
            motorCmd[legID*3 + 0].Kd = 0.8f;
            motorCmd[legID*3 + 1].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 1].Kp = 0.8f;
            motorCmd[legID*3 + 1].Kd = 0.8f;
            motorCmd[legID*3 + 2].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 2].Kp = 0.8f;
            motorCmd[legID*3 + 2].Kd = 0.8f;
        }

        /**
         * @brief 设置所有腿的稳定模式增益
         */
        void setStableGain() {
            for (int i = 0; i < 4; ++i) {
                setStableGain(i);
            }
        }

        /**
         * @brief 设置单条腿的摆动模式增益
         * @param legID 腿索引（0-3）
         */
        void setSwingGain(int legID) {
            if (legID < 0 || legID >= 4) {
                printf("[LowlevelCmd] 错误：legID=%d（应在0-3之间）\n", legID);
                return;
            }
            motorCmd[legID*3 + 0].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 0].Kp = 1.8f;
            motorCmd[legID*3 + 0].Kd = 0.2f;
            motorCmd[legID*3 + 1].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 1].Kp = 1.8f;
            motorCmd[legID*3 + 1].Kd = 0.2f;
            motorCmd[legID*3 + 2].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            motorCmd[legID*3 + 2].Kp = 1.0f;
            motorCmd[legID*3 + 2].Kd = 0.2f;
        }

        /**
         * @brief 设置所有腿的摆动模式增益（补充全局版本，保持一致性）
         */
        void setSwingGain() {
            for (int i = 0; i < 4; ++i) {
                setSwingGain(i);
            }
        }
    };
}  // namespace UserLowlevel

#endif  // LOWLEVELCMD_H