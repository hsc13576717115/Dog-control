#ifndef STATE_STEPTEST_H
#define STATE_STEPTEST_H

#include "FSMState.h"
#include "control/CtrlComponents.h"
#include <Eigen/Dense>

class State_StepTest : public FSMState {
public:
    State_StepTest(CtrlComponents* ctrlComp);
    ~State_StepTest() = default;

    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;
    void calcTau();

private:
    int _transitionCount;          // 状态过渡计数器
    bool _isCalibrated;            // 校准状态标志
    Eigen::Vector3d _calibOff;     // 校准偏移量
    Eigen::VectorXd _initMotorQ;   // 初始电机角度(12维)
    Eigen::Vector3d _initFootPos[4]; // 四腿初始足端位置
    Eigen::Vector3d _lastLegQ[4];  // 四腿上一时刻关节角度
    double _startTime;             // 状态开始时间
    bool _isJumpCompleted;  // 标记是否完成一次跳跃
    bool _isCycleEnded;

    // 机械参数（类内引用）
    static constexpr double L0 = 0.108;    // 髋关节到大腿长度
    static constexpr double L1 = 0.225;    // 大腿长度
    static constexpr double L2 = 0.255;    // 小腿长度

    // 关节限位（类内引用）
    static constexpr double Q0_LIMIT_MIN = -2.60;
    static constexpr double Q0_LIMIT_MAX =  2.60;
    static constexpr double Q1_LIMIT_MIN = -6.50;
    static constexpr double Q1_LIMIT_MAX =  6.50;
    static constexpr double Q2_LIMIT_MIN = -2.30;
    static constexpr double Q2_LIMIT_MAX =  2.30;

    static constexpr double HIP_JOINT_FIXED = 0.0;
};

#endif // STATE_STEPTEST_H