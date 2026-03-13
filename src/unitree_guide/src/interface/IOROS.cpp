/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include "interface/WirelessHandle.h"  // 无线手柄头文件
#include "message/LowlevelCmd.h"       // 引入UserLowlevel命名空间
#include "message/LowlevelState.h"     // 修正：添加State头文件，避免MotorState成员未识别
#include <iostream>
#include <unistd.h>
#include <csignal>

// ROS关闭信号处理（Ctrl+C触发）
void RosShutDown(int sig){
	ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}

// 构造函数：初始化ROS通信+输入设备
IOROS::IOROS():IOInterface(){
    std::cout << "ROS Gazebo仿真控制接口初始化" << std::endl;
    // 读取机器人名称（默认unitree_go1，适配Gazebo话题）
    if (!ros::param::get("/robot_name", _robot_name)) {
        _robot_name = "unitree_go1";
        std::cout << "未设置/robot_name参数，使用默认值: " << _robot_name << std::endl;
    } else {
        std::cout << "robot_name: " << _robot_name << std::endl;
    }

    // 1. 初始化ROS订阅（接收Gazebo的电机/IMU状态）
    initRecv();
    ros::AsyncSpinner subSpinner(1);  // 1个线程处理订阅回调（非阻塞）
    subSpinner.start();
    usleep(300000);  // 等待300ms，确保订阅器启动完成

    // 2. 初始化ROS发布（发送电机控制命令到Gazebo）
    initSend();   

    // 3. 注册Ctrl+C信号处理（安全关闭ROS）
    signal(SIGINT, RosShutDown);

    // 4. 选择输入设备（二选一，注释一个启用另一个）
    // 选项A：键盘控制（默认，无需额外依赖）
    cmdPanel = new KeyBoard();
    std::cout << "输入设备：键盘（按Ctrl+C退出）" << std::endl;

    // 选项B：Unitree无线手柄（需安装unitree_legged_sdk，启用前注释选项A）
    // cmdPanel = new WirelessHandle();
    // std::cout << "输入设备：Unitree无线手柄（确保USB接收器已连接）" << std::endl;
}

IOROS::~IOROS(){
    delete cmdPanel;  // 释放输入设备资源
    ros::shutdown();  // 关闭ROS节点
}

// 发送+接收主逻辑（带UserLowlevel::前缀，匹配命名空间）
void IOROS::sendRecv(const UserLowlevel::LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);          // 发送控制命令到Gazebo
    recvState(state);      // 从Gazebo接收电机/IMU状态
    // 读取输入设备指令（键盘/手柄的命令和摇杆值）
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

// 修正：用户命令→ROS消息转换（适配UserLowlevel命名空间）
void IOROS::sendCmd(const UserLowlevel::LowlevelCmd *lowCmd){
    for(int i = 0; i < 12; ++i){
        _lowCmd.motorCmd[i].mode = lowCmd->motorCmd[i].mode;  // 控制模式（10=复合）
        _lowCmd.motorCmd[i].q = lowCmd->motorCmd[i].q;        // 目标位置（弧度）
        _lowCmd.motorCmd[i].dq = lowCmd->motorCmd[i].dq;      // 目标速度（弧度/秒）
        _lowCmd.motorCmd[i].tau = lowCmd->motorCmd[i].tau;    // 目标力矩（N·m）
        _lowCmd.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;      // 速度增益
        _lowCmd.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;      // 位置增益
    }

    // 发布命令到Gazebo的12个电机控制器（话题匹配Gazebo配置）
    for(int m = 0; m < 12; ++m){
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    ros::spinOnce();  // 处理ROS回调（非阻塞，避免卡住）
}

// 修正：从Gazebo接收状态（适配MotorState新成员）
void IOROS::recvState(LowlevelState *state){
    // 电机状态：Gazebo→用户状态（填充temp和fault的默认值）
    for(int i = 0; i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;        // 位置
        state->motorState[i].dq = _lowState.motorState[i].dq;      // 速度
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;  // 估计力矩
        // 仿真模式：Gazebo无实际温度/错误码，填充默认值
        state->motorState[i].temp = 30;  // 模拟正常温度（30℃）
        state->motorState[i].fault = 0;   // 模拟无错误
    }

    // IMU状态：Gazebo→用户状态（四元数顺序：w→x→y→z）
    state->imu.quaternion[0] = _lowState.imu.quaternion[0];  // x
    state->imu.quaternion[1] = _lowState.imu.quaternion[1];  // y
    state->imu.quaternion[2] = _lowState.imu.quaternion[2];  // z
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];  // w
    // 陀螺仪和加速度计数据（直接从Gazebo获取）
    for(int i = 0; i < 3; ++i){
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
    }
}

// 初始化ROS发布器（话题路径匹配Gazebo电机控制器）
void IOROS::initSend(){
    std::string prefix = "/" + _robot_name + "_gazebo/";  // Gazebo话题前缀
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "FR_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "FR_thigh_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "FR_calf_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "FL_hip_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "FL_thigh_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "FL_calf_controller/command", 1);
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "RR_hip_controller/command", 1);
    _servo_pub[7] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "RR_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "RR_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "RL_hip_controller/command", 1);
    _servo_pub[10] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "RL_thigh_controller/command", 1);
    _servo_pub[11] = _nm.advertise<unitree_legged_msgs::MotorCmd>(prefix + "RL_calf_controller/command", 1);
}

// 初始化ROS订阅器（接收Gazebo的状态话题）
void IOROS::initRecv(){
    // 订阅躯干IMU话题（Gazebo发布的IMU数据）
    _imu_sub = _nm.subscribe("/trunk_imu", 1, &IOROS::imuCallback, this);
    // 订阅12个电机的状态话题
    std::string prefix = "/" + _robot_name + "_gazebo/";
    _servo_sub[0] = _nm.subscribe(prefix + "FR_hip_controller/state", 1, &IOROS::FRhipCallback, this);
    _servo_sub[1] = _nm.subscribe(prefix + "FR_thigh_controller/state", 1, &IOROS::FRthighCallback, this);
    _servo_sub[2] = _nm.subscribe(prefix + "FR_calf_controller/state", 1, &IOROS::FRcalfCallback, this);
    _servo_sub[3] = _nm.subscribe(prefix + "FL_hip_controller/state", 1, &IOROS::FLhipCallback, this);
    _servo_sub[4] = _nm.subscribe(prefix + "FL_thigh_controller/state", 1, &IOROS::FLthighCallback, this);
    _servo_sub[5] = _nm.subscribe(prefix + "FL_calf_controller/state", 1, &IOROS::FLcalfCallback, this);
    _servo_sub[6] = _nm.subscribe(prefix + "RR_hip_controller/state", 1, &IOROS::RRhipCallback, this);
    _servo_sub[7] = _nm.subscribe(prefix + "RR_thigh_controller/state", 1, &IOROS::RRthighCallback, this);
    _servo_sub[8] = _nm.subscribe(prefix + "RR_calf_controller/state", 1, &IOROS::RRcalfCallback, this);
    _servo_sub[9] = _nm.subscribe(prefix + "RL_hip_controller/state", 1, &IOROS::RLhipCallback, this);
    _servo_sub[10] = _nm.subscribe(prefix + "RL_thigh_controller/state", 1, &IOROS::RLthighCallback, this);
    _servo_sub[11] = _nm.subscribe(prefix + "RL_calf_controller/state", 1, &IOROS::RLcalfCallback, this);
}

// IMU回调函数：Gazebo IMU数据→_lowState
void IOROS::imuCallback(const sensor_msgs::Imu & msg)
{ 
    _lowState.imu.quaternion[0] = msg.orientation.w;  // w
    _lowState.imu.quaternion[1] = msg.orientation.x;  // x
    _lowState.imu.quaternion[2] = msg.orientation.y;  // y
    _lowState.imu.quaternion[3] = msg.orientation.z;  // z
    // 陀螺仪数据（rad/s）
    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    // 加速度计数据（m/s²）
    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}

// 12个电机的回调函数：Gazebo状态→_lowState
void IOROS::FRhipCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[0] = msg; }
void IOROS::FRthighCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[1] = msg; }
void IOROS::FRcalfCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[2] = msg; }
void IOROS::FLhipCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[3] = msg; }
void IOROS::FLthighCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[4] = msg; }
void IOROS::FLcalfCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[5] = msg; }
void IOROS::RRhipCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[6] = msg; }
void IOROS::RRthighCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[7] = msg; }
void IOROS::RRcalfCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[8] = msg; }
void IOROS::RLhipCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[9] = msg; }
void IOROS::RLthighCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[10] = msg; }
void IOROS::RLcalfCallback(const unitree_legged_msgs::MotorState& msg) { _lowState.motorState[11] = msg; }

#endif  // COMPILE_WITH_ROS