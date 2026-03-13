#ifndef STATE_FIXEDSTAND_H
#define STATE_FIXEDSTAND_H

#include "FSMState.h"
#include <Eigen/Core>

// 类型定义（与SwingTest保持一致，避免类型冲突）
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 12, 1> Vec12;

// 定义站立模式枚举
enum StandMode {
    NORMAL_STAND = 0,  // 正常站立模式
    CROUCH = 1         // 匍匐模式
};

class State_FixedStand : public FSMState {
public:
    // 构造函数（接收控制组件指针）
    State_FixedStand(CtrlComponents *ctrlComp);
    // 析构函数（默认空实现）
    ~State_FixedStand() = default;

    // 状态生命周期函数（重写基类）
    void enter() override;   // 进入状态：初始化+IK计算目标角度
    void run() override;     // 运行状态：平滑过渡+状态打印
    void exit() override;    // 退出状态：重置进度
    FSMStateName checkChange() override;  // 状态切换判断

private:
    // 移除冗余的_qCalib（已统一用L型姿态校准）
    const Vec3 _pHip2B;          // 髋部偏移（与SwingTest/IOSDK统一：0.1525, -0.0565, 0.0）
    bool _debugMotorEnable;      // 单电机调试模式开关
    int _debugMotorID;           // 调试电机ID（0~11）
    int _duration;               // 平滑过渡周期（控制周期数，默认500=5秒）
    float _percent;              // 过渡进度（0.0~1.0）
    Vec12 _targetPos;            // 12轴目标角度（通过IK计算得到）
    double _startPos[12];        // 进入状态时的初始角度（用于平滑过渡）
    StandMode _currentMode;      // 当前模式（正常站立或匍匐）
    StandMode _lastMode;         // 上次模式（用于记录）
};

#endif  // STATE_FIXEDSTAND_H



