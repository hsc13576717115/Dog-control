/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <iomanip>
#include <mutex>  // 用于线程安全
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// 机器人控制相关头文件
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"
#include "interface/IOSDK.h"
#include "message/LowlevelState.h"  // 包含IMU结构体定义

bool running = true;

// 捕获Ctrl+C信号，用于安全退出
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

// 设置进程为实时调度
void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

// 全局变量：存储最新的IMU数据（线程安全）
sensor_msgs::Imu latest_imu_data;
std::mutex imu_mutex;  // 保护IMU数据的互斥锁

// 外部IMU话题回调函数：接收/imu数据并保存
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex);  // 加锁防止数据竞争
    latest_imu_data = *msg;
}

int main(int argc, char **argv)
{
    /* 设置实时进程和打印格式 */
    setProcessScheduler();
    std::cout << std::fixed << std::setprecision(3);

    // 初始化ROS节点
    ros::init(argc, argv, "unitree_external_imu_controller");
    ros::NodeHandle nh;
    std::cout << "[IMU] 启动ROS节点，开始订阅外部/imu话题..." << std::endl;

    // 订阅外部IMU话题（来自fdilink_ahrs）
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
        "/imu", 2000, imuCallback  // 话题名：/imu，队列长度10
    );

    // 机器人控制核心逻辑初始化
    IOInterface *ioInter = new IOSDK();  // 实物接口
    CtrlPlatform ctrlPlat = CtrlPlatform::REALROBOT;  // 实物模式

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002;  // 500Hz控制频率
    ctrlComp->running = &running;

#ifdef ROBOT_TYPE_A1
    ctrlComp->robotModel = new A1Robot();
#else
    ctrlComp->robotModel = new Go1Robot(); 
#endif

    // 步态生成器配置
    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); 

    // 生成控制相关对象（包括Estimator等）
    ctrlComp->geneObj();
    ControlFrame ctrlFrame(ctrlComp);
    signal(SIGINT, ShutDown);

    // 检查lowState是否初始化成功
    if (ctrlComp->lowState == nullptr) {
        std::cerr << "[ERROR] lowState初始化失败，无法获取IMU存储对象！" << std::endl;
        delete ctrlComp;
        delete ioInter;
        return -1;
    }

    // 主循环：控制逻辑 + 外部IMU数据注入
    ros::Rate loop_rate(1000);  // 与控制频率一致（500Hz）
    while (running && ros::ok())
    {
        // 1. 读取最新的外部IMU数据（线程安全）
        sensor_msgs::Imu current_imu;
        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            current_imu = latest_imu_data;
        }

        // 2. 将外部IMU数据转换并更新到lowState->imu（核心步骤）
        IMU& robot_imu = ctrlComp->lowState->imu;  // 引用机器人内部IMU结构体

        // 2.1 四元数（w, x, y, z）：double转float，匹配IMU结构体格式
        robot_imu.quaternion[0] = static_cast<float>(current_imu.orientation.w);  // w
        robot_imu.quaternion[1] = static_cast<float>(current_imu.orientation.x);  // x
        robot_imu.quaternion[2] = static_cast<float>(current_imu.orientation.y);  // y
        robot_imu.quaternion[3] = static_cast<float>(current_imu.orientation.z);  // z

        // 2.2 角速度（x, y, z）：rad/s，double转float
        robot_imu.gyroscope[0] = static_cast<float>(current_imu.angular_velocity.x);
        robot_imu.gyroscope[1] = static_cast<float>(current_imu.angular_velocity.y);
        robot_imu.gyroscope[2] = static_cast<float>(current_imu.angular_velocity.z);

        // 2.3 线加速度（x, y, z）：m/s²，double转float
        robot_imu.accelerometer[0] = static_cast<float>(current_imu.linear_acceleration.x);
        robot_imu.accelerometer[1] = static_cast<float>(current_imu.linear_acceleration.y);
        robot_imu.accelerometer[2] = static_cast<float>(current_imu.linear_acceleration.z);

        // 3. 运行机器人控制逻辑（此时控制逻辑会使用更新后的外部IMU数据）
        ctrlFrame.run();

        // 4. 处理ROS回调（包括IMU订阅）
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 资源释放
    delete ctrlComp;  // 内部会释放lowState、ioInter等
    std::cout << "[IMU] 程序退出，资源已释放" << std::endl;
    return 0;
}