// #ifndef IOSDK_H
// #define IOSDK_H

// #include <vector>
// #include <set>
// #include <array>
// #include <Eigen/Core>
// #include "message/LowlevelState.h"
// #include "interface/IOInterface.h"
// #include "message/LowlevelCmd.h"
// #include "unitreeMotor/unitreeMotor.h"
// #include "serialPort/SerialPort.h"
// #include "interface/CmdPanel.h"

// typedef Eigen::Matrix<double, 3, 1> Vec3;

// class IOSDK : public IOInterface {
// public:
//     IOSDK();
//     ~IOSDK();
//     void sendRecv(const UserLowlevel::LowlevelCmd *cmd, LowlevelState *state) override;
//     bool isCalibrated() const { return _isCalibrated; }
//     const std::array<float, 12>& getCalibOffset() const { return _calibOffset; }
// private:
//     std::vector<SerialPort*> _serials;
//     MotorCmd  _motorCmd[12];
//     MotorData _motorData[12];
//     CmdPanel* _cmdPanel;
//     SerialLowState _serialState;
//     std::set<int> _activeLegs;
//     std::array<float, 12> _calibOffset;  // 校准偏移：让qUser=0对应L型姿态电机角度
//     bool _isCalibrated = false;
//     const UserCommand _calibTriggerKey = UserCommand::L1_X;  // 校准触发键
//     const float _calibPromptInterval = 5.0f;
//     double _lastCalibPromptTime = 0.0;

//     // 减速比（全局）
//     static constexpr float GEAR_HIP = 1.0f;
//     static constexpr float GEAR_THIGH = 1.0f;
//     static constexpr float GEAR_CALF = 2.0f;
//     static constexpr float BASE_GEAR_RATIO = 6.33f;  // 电机基础减速比（GO-M8010-6）
//     static constexpr float CALF_TOTAL_GEAR = BASE_GEAR_RATIO * GEAR_CALF;  // 小腿总减速比

// #ifdef COMPILE_WITH_MOVE_BASE
//     ros::NodeHandle _nh;
//     ros::Publisher _jointPub;
//     sensor_msgs::JointState _jointState;
// #endif
// };

// #endif  // IOSDK_H

#ifndef IOSDK_H
#define IOSDK_H

#include <vector>
#include <set>
#include <array>
#include <Eigen/Core>
#include <ros/ros.h>                  // ROS头文件
#include <sensor_msgs/Joy.h>          // Joy消息头文件
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "message/LowlevelCmd.h"
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"
#include "common/enumClass.h"         // 引入原项目的UserCommand枚举
#include "interface/CmdPanel.h"       // 引入原项目的UserValue结构体（仅用定义，不使用CmdPanel类）

typedef Eigen::Matrix<double, 3, 1> Vec3;

class IOSDK : public IOInterface {
public:
    IOSDK();
    ~IOSDK();
    void sendRecv(const UserLowlevel::LowlevelCmd *cmd, LowlevelState *state) override;
    bool isCalibrated() const { return _isCalibrated; }
    const std::array<float, 12>& getCalibOffset() const { return _calibOffset; }

    UserCommand _currentUserCmd;          
    UserValue _currentUserValue;  

private:
    ros::NodeHandle _nh;                  // ROS节点句柄
    ros::Subscriber _joySub;              // Joy话题订阅者
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);  // Joy回调函数

        

    std::vector<SerialPort*> _serials;
    MotorCmd  _motorCmd[12];
    MotorData _motorData[12];
    std::set<int> _activeLegs;
    std::array<float, 12> _calibOffset;  // 校准偏移
    bool _isCalibrated = false;
    const UserCommand _calibTriggerKey = UserCommand::L1_X;  // 校准触发键（与原项目枚举匹配）
    const float _calibPromptInterval = 5.0f;
    double _lastCalibPromptTime = 0.0;

    // 减速比定义（保留）
    static constexpr float GEAR_HIP = 1.0f;
    static constexpr float GEAR_THIGH = 1.0f;
    static constexpr float GEAR_CALF = 2.0f;
    static constexpr float BASE_GEAR_RATIO = 6.33f;
    static constexpr float CALF_TOTAL_GEAR = BASE_GEAR_RATIO * GEAR_CALF;

    // 删除原键盘相关成员（不再使用）
    // CmdPanel* _cmdPanel;
    // SerialLowState _serialState;

#ifdef COMPILE_WITH_MOVE_BASE
    ros::Publisher _jointPub;             // 关节状态发布者
    sensor_msgs::JointState _jointState;  // 关节状态消息
#endif
};

#endif  // IOSDK_H