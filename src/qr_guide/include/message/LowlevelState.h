/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include "common/enumClass.h"

// 修正：在原有MotorState中添加temp（温度）和fault（错误码）成员
struct MotorState
{
    unsigned int mode;  // 原有：电机控制模式
    float q;            // 原有：位置（弧度）
    float dq;           // 原有：速度（弧度/秒）
    float ddq;          // 原有：加速度（弧度/秒²）
    float tauEst;       // 原有：估计力矩（N·m）
    int temp;           // 新增：电机温度（℃，适配IOSDK/IOROS）
    int fault;          // 新增：错误码（0=无错误，适配IOSDK/IOROS）

    // 修正：构造函数初始化新增成员（避免未定义值）
    MotorState(){
        mode = 0;       // 补充mode的初始化（原有构造未初始化）
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
        temp = 0;       // 新增成员初始化
        fault = 0;      // 新增成员初始化
    }
};

struct IMU
{
    float quaternion[4];    // w, x, y, z（原有）
    float gyroscope[3];     // 原有：陀螺仪（rad/s）
    float accelerometer[3]; // 原有：加速度计（m/s²）

    IMU(){
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }

    // 原有方法：获取旋转矩阵
    RotMat getRotMat() const {
        Quat quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    // 原有方法：获取加速度（机体坐标系）
    Vec3 getAcc() const {
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    // 原有方法：获取陀螺仪数据（机体坐标系）
    Vec3 getGyro() const {
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    // 原有方法：获取四元数
    Quat getQuat() const {
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};

struct LowlevelState
{
    IMU imu;                // 原有：IMU数据
    MotorState motorState[12]; // 原有：12个电机状态（已包含新增成员）
    UserCommand userCmd;    // 原有：用户命令（手柄/键盘）
    UserValue userValue;    // 原有：用户输入值（摇杆）

    // 原有方法：获取所有腿的位置（3x4矩阵）
    Vec34 getQ() const {
        Vec34 qLegs;
        for(int i(0); i < 4; ++i){
            qLegs.col(i)(0) = motorState[3*i    ].q;
            qLegs.col(i)(1) = motorState[3*i + 1].q;
            qLegs.col(i)(2) = motorState[3*i + 2].q;
        }
        return qLegs;
    }

    // 原有方法：获取所有腿的速度（3x4矩阵）
    Vec34 getQd() const {
        Vec34 qdLegs;
        for(int i(0); i < 4; ++i){
            qdLegs.col(i)(0) = motorState[3*i    ].dq;
            qdLegs.col(i)(1) = motorState[3*i + 1].dq;
            qdLegs.col(i)(2) = motorState[3*i + 2].dq;
        }
        return qdLegs;
    }

    // 原有方法：获取旋转矩阵（全局坐标系）
    RotMat getRotMat() const {
        return imu.getRotMat();
    }

    // 原有方法：获取加速度（机体坐标系）
    Vec3 getAcc() const {
        return imu.getAcc();
    }

    // 原有方法：获取陀螺仪数据（机体坐标系）
    Vec3 getGyro() const {
        return imu.getGyro();
    }

    // 原有方法：获取加速度（全局坐标系）
    Vec3 getAccGlobal() const {
        return getRotMat() * getAcc();
    }

    // 原有方法：获取陀螺仪数据（全局坐标系）
    Vec3 getGyroGlobal() const {
        return getRotMat() * getGyro();
    }

    // 原有方法：获取偏航角（yaw，全局坐标系）
    double getYaw() const {
        return rotMatToRPY(getRotMat())(2);
    }

    // 原有方法：获取偏航角速度（全局坐标系）
    double getDYaw() const {
        return getGyroGlobal()(2);
    }

    // 原有方法：设置所有电机的位置
    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorState[i].q = q(i);
        }
    }
};

#endif  // LOWLEVELSTATE_HPP
