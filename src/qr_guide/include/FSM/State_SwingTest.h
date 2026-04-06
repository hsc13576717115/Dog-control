#ifndef STATE_SWINGTEST_H
#define STATE_SWINGTEST_H

#pragma once
#include "FSMState.h"
#include <Eigen/Core>
#include "interface/IOSDK.h"
// 类型定义（与FixedStand统一，避免类型冲突）
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 12, 1> Vec12;

// 全局减速比定义（电机角度转换用）
static constexpr float GEAR_HIP = 1.0f;
static constexpr float GEAR_THIGH = 1.0f;
static constexpr float GEAR_CALF = 2.0f;

// 统一clamp函数（全局可见，FixedStand也可使用）
template <typename T>
T clamp(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

class State_SwingTest : public FSMState {
public:
    // 构造函数/析构函数
    State_SwingTest(CtrlComponents *ctrlComp);
    ~State_SwingTest() = default;

    // 状态生命周期函数
    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    // 成员变量（与你原代码一致）
    const Vec3 _pHip2B;          // 髋部偏移（0.1525, -0.0565, 0.0）
    const double _xRange;        // X方向控制范围（0.008m）
    const double _zRange;        // Z方向控制范围（0.006m）
    bool _firstRun;              // 首次运行标记
    float _transPct;             // 过渡进度
    Vec3 _standQ;                // 站立姿态角度
    Vec3 _enterQ;                // 进入状态时的角度
    Vec3 _prevQ;                 // 上一状态角度
    Vec3 _standFoot_B;           // 站立姿态足端（BODY系）
    Vec3 _zeroFoot_B;            // L型姿态足端（BODY系）
    Vec3 _standXZ;               // 站立姿态足端（轴心系）
    Vec3 _zeroXZ;                // L型姿态足端（轴心系）
    Vec3 _initXZ;                // 初始足端坐标（轴心系）
    Vec3 _goalXZ;                // 目标足端坐标（轴心系）
    Vec3 _smoothXZ;              // 平滑后的目标坐标
    Vec3 _initPos;               // 初始位置（备用）
    Vec3 _posGoal;               // 目标位置（备用）
    Vec12 _startPos;             // 初始12轴角度
    Vec3 smoothGoal_;            // 平滑目标（备用）

    // 静态常量：过渡周期（FixedStand也可引用）
    static constexpr int _transitionDuration = 50;
};

// ================== 关键：添加 ikCheckAxis 和 fkCheckAxis 的全局声明 ==================
// 1. 轴心坐标系FK：角度→足端坐标（原点=大腿电机轴心）
// 参数：qUser - [髋角, 大腿相对角度, 小腿相对角度]（相对L型姿态）
// 返回：足端坐标（轴心系，x=向前，z=向上）
// Vec3 fkCheckAxis(const Vec3 &qUser);

// 2. 轴心坐标系IK：足端坐标→角度（原点=大腿电机轴心）
// 参数：pDes - 目标足端坐标（轴心系，x=向前，z=向上）
// 返回：关节角度（相对L型姿态，[髋角, 大腿角度, 小腿角度]）
Vec3 ikCheckAxis(const Vec3 &pDes, int leg = 0);
Vec3 fkCheckAxis(const Vec3 &qUser, int leg = 0);

// 原有函数声明（保持不变）
Vec3 fkCheck(const Vec3 &qUser, const Vec3 &hip2B);
Vec3 ikCheck(const Vec3 &pDes_B, const Vec3 &hip2B);
void prt(const char *s, const Vec3 &v);

enum LegType { RIGHT_LEG, LEFT_LEG };

Vec3 fkLeg(const Vec3& qUser, LegType leg);
Vec3 ikLeg(const Vec3& pDes, LegType leg);

#endif  // STATE_SWINGTEST_H