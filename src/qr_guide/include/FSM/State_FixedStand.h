/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef STATE_FIXEDSTAND_H
#define STATE_FIXEDSTAND_H

#include <array>
#include <memory>

#include "FSM/FSMState.h"

class Estimator;
class QuadrupedRobot;
class BalanceCtrl;

/**
 * @brief 固定站立状态（力控版）。
 *
 * 采用与 State_Trotting 一致的力控架构：
 *   1. Body PD 控制器 → 期望加速度 (ddPcd, dWbd)
 *   2. BalanceCtrl QP → 四条支撑腿足端力
 *   3. Jacobian 转置 → 关节力矩
 *   4. 复合模式输出 → tau + q + qd
 *
 * 进入时先用位置控制将腿摆到目标位置（过渡平滑），
 * 然后切换为全力控模式。
 */
class State_FixedStand : public FSMState {
public:
    explicit State_FixedStand(CtrlComponents* ctrlComp);
    ~State_FixedStand() override = default;

    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    // 位置控制过渡阶段
    void runPositionTransition();

    // 力控阶段
    void runForceControl(float blend);
    void applyTorqueSafety();

    Vec3 clampJointAngles(const Vec3& q) const;
    float transitionBlend() const;
    Vec3 computeBaseStandFootTargetInHip(int leg) const;
    Vec3 computeCompensatedFootTargetInHip(int leg, const Vec3& base_target_in_hip) const;

    // 共享组件
    Estimator* _est = nullptr;
    BalanceCtrl* _balCtrl = nullptr;
    QuadrupedRobot* _robModel = nullptr;

    // 过渡阶段参数
    int _duration = 500;
    float _percent = 0.0f;
    std::array<Vec3, qr_guide::NumLeg> _startFeetInHip{};
    std::array<Vec3, qr_guide::NumLeg> _targetFeetInHip{};
    std::array<Vec3, qr_guide::NumLeg> _targetFeetInBody{};

    // 力控状态
    Vec3 _pcd = Vec3::Zero();
    Vec3 _ddPcd = Vec3::Zero();
    Vec3 _dWbd = Vec3::Zero();
    Vec34 _forceFeetGlobal = Vec34::Zero();
    Vec34 _forceFeetBody = Vec34::Zero();
    Vec34 _posFeet2BGlobal = Vec34::Zero();
    Vec12 _tau = Vec12::Zero();
    Vec12 _tauPrev = Vec12::Zero();
    RotMat _B2G_RotMat = RotMat::Identity();
    RotMat _G2B_RotMat = RotMat::Identity();

    // PD 增益（从 YAML force_control 参数读取）
    Mat3 _Kpp = Mat3::Zero();
    Mat3 _Kdp = Mat3::Zero();
    Mat3 _Kpw = Mat3::Zero();
    Mat3 _Kdw = Mat3::Zero();

    bool _forceControlActive = false;
};

#endif  // STATE_FIXEDSTAND_H
