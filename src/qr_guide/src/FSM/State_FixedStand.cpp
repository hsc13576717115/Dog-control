/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_FixedStand.h"
#include "FSM/StateMotorParams.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "control/BalanceCtrl.h"
#include "control/Estimator.h"
#include "common/mathTools.h"
#include "common/unitreeRobot.h"

namespace {

template<typename T>
T clampValue(T value, T min_value, T max_value) {
    return std::min(std::max(value, min_value), max_value);
}

}  // namespace

State_FixedStand::State_FixedStand(CtrlComponents* ctrlComp)
    : FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"),
      _est(ctrlComp->estimator.get()),
      _balCtrl(ctrlComp->balCtrl.get()),
      _robModel(ctrlComp->robotModel.get()) {
    for (auto& target : _targetFeetInBody) {
        target.setZero();
    }

    const auto& f = ctrlComp->parameters.force;
    _Kpp = f.kp_body_xyz.asDiagonal();
    _Kdp = f.kd_body_xyz.asDiagonal();
    _Kpw = f.kp_body_rpy.asDiagonal();
    _Kdw = f.kd_body_rpy.asDiagonal();
}

void State_FixedStand::enter() {
    _duration = std::max(1, _ctrlComp->parameters.stand.entry_duration);
    _percent = 0.0f;
    _forceControlActive = false;
    _tauPrev.setZero();

    std::cout << "\n========== [FixedStand] ENTER ==========" << std::endl;

    for (int leg = 0; leg < 4; ++leg) {
        fsm_motor_params::ApplyLegProfile(_lowCmd, leg, fsm_motor_params::kFixedStandProfile);

        const Vec3 q = _lowState->getQ().col(leg);
        _startFeetInHip[leg] = _robModel->forwardKinematics(q, leg, FrameType::HIP);
        _targetFeetInHip[leg] = computeBaseStandFootTargetInHip(leg);
        _targetFeetInBody[leg] =
            _ctrlComp->parameters.hip_mounts_in_body[leg] + _targetFeetInHip[leg];

        // IK 解算目标姿态，用于调试
        Vec3 q_ik = _robModel->inverseKinematics(_targetFeetInHip[leg], leg, FrameType::HIP);

        const char* leg_name = (leg == 0) ? "FR" : (leg == 1) ? "FL" : (leg == 2) ? "RR" : "RL";
        std::cout << std::fixed << std::setprecision(3)
                  << "[" << leg_name << "] init_q=" << q.transpose()
                  << " target_hip=" << _targetFeetInHip[leg].transpose()
                  << " ik_q=" << q_ik.transpose()
                  << " start_hip=" << _startFeetInHip[leg].transpose()
                  << std::endl;
    }

    // 初始化力控期望状态
    _pcd = _est->getPosition();
    _pcd(2) = _ctrlComp->parameters.stand.body_height;

    const Vec3 rpy = rotMatToRPY(_lowState->getRotMat());
    std::cout << "[FixedStand] init pos=" << _pcd.transpose()
              << " rpy=" << rpy.transpose()
              << " body_height=" << _ctrlComp->parameters.stand.body_height
              << "\n========================================\n"
              << std::endl;

    _ctrlComp->setAllStance();
}

void State_FixedStand::run() {
    _percent += 1.0f / static_cast<float>(_duration);
    if (_percent > 1.0f) {
        _percent = 1.0f;
    }

    const float blend = transitionBlend();

    // 第一阶段：位置控制过渡（把腿摆到目标位置）
    if (blend < 1.0f) {
        runPositionTransition();
    }

    // 第二阶段：力控接管（blend 从 0→1 时力矩权重递增）
    runForceControl(blend);
}

void State_FixedStand::runPositionTransition() {
    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 base_target =
            (1.0f - _percent) * _startFeetInHip[leg] + _percent * _targetFeetInHip[leg];
        const Vec3 compensated_target = computeCompensatedFootTargetInHip(leg, base_target);
        _targetFeetInBody[leg] =
            _ctrlComp->parameters.hip_mounts_in_body[leg] + compensated_target;

        Vec3 q = _robModel->inverseKinematics(compensated_target, leg, FrameType::HIP);
        q = clampJointAngles(q);

        for (int joint = 0; joint < 3; ++joint) {
            const int id = leg * 3 + joint;
            _lowCmd->motorCmd[id].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            _lowCmd->motorCmd[id].dq = 0.0f;
            _lowCmd->motorCmd[id].q = q(joint);
            _lowCmd->motorCmd[id].tau = 0.0f;
        }
    }
}

void State_FixedStand::runForceControl(float blend) {
    // 1. 获取机身状态
    const Vec3 posBody = _est->getPosition();
    const Vec3 velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    // 2. 目标：保持当前 x,y 水平位置，固定高度，水平姿态
    _pcd(0) = posBody(0);
    _pcd(1) = posBody(1);
    _pcd(2) = _ctrlComp->parameters.stand.body_height;

    const Vec3 posError = _pcd - posBody;
    const Vec3 velError = -velBody;

    // 隔离测试：先只验证纯高度支撑，姿态控制暂时关闭
    _ddPcd.setZero();
    _ddPcd(2) = _Kpp(2, 2) * posError(2) + _Kdp(2, 2) * velError(2);
    _dWbd.setZero();

    const auto& f = _ctrlComp->parameters.force;
    _ddPcd(0) = saturation(_ddPcd(0), f.acc_xy_sat);
    _ddPcd(1) = saturation(_ddPcd(1), f.acc_xy_sat);
    _ddPcd(2) = saturation(_ddPcd(2), f.acc_z_sat);
    _dWbd(0) = saturation(_dWbd(0), f.w_roll_pitch_sat);
    _dWbd(1) = saturation(_dWbd(1), f.w_roll_pitch_sat);
    _dWbd(2) = saturation(_dWbd(2), f.w_yaw_sat);

    // 3. QP 力分配（4 条腿全支撑）
    VecInt4 contact = VecInt4::Ones();
    _forceFeetGlobal = _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, contact);
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    // 4. Jacobian 转置映射力矩（注意符号：如果实测机身下塌，取负）
    Vec12 q = vec34ToVec12(_lowState->getQ());
    _tau = -_robModel->getTau(q, _forceFeetBody);

    // 5. 力矩安全限幅
    applyTorqueSafety();

    // 6. 过渡平滑：位置控制阶段 blend<1 时力矩权重递增
    _tau *= blend;

    // 7. 输出复合命令
    for (int leg = 0; leg < 4; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            const int id = leg * 3 + joint;
            _lowCmd->motorCmd[id].tau = _tau(id);
        }
        _lowCmd->setStableGain(leg);
    }

    // ===== 力控调试输出（每 50 帧 ≈ 0.1s 打印一次）=====
    static int dbg_cnt = 0;
    if (++dbg_cnt % 50 == 0) {
        const Vec3 rpy = rotMatToRPY(_B2G_RotMat);
        const double total_fz = _forceFeetGlobal.row(2).sum();
        const double gravity = _robModel->getRobMass() * 9.81;
        // 修复后 Fz_sum 应为正值（地面向上推），约等于 gravity + m*ddPcd.z
        std::cout << std::fixed << std::setprecision(3)
                  << "[FixedStand][dbg] blend=" << blend
                  << " posErr=" << posError.transpose()
                  << " rpy=" << rpy.transpose()
                  << " ddPcd=" << _ddPcd.transpose()
                  << " dWbd=" << _dWbd.transpose()
                  << " Fz_sum=" << total_fz << "/" << gravity
                  << " tau_FR=" << _tau.segment(0,3).transpose()
                  << " tau_FL=" << _tau.segment(3,3).transpose()
                  << std::endl;
    }
}

void State_FixedStand::applyTorqueSafety() {
    const auto& f = _ctrlComp->parameters.force;
    const double max_delta = f.tau_rate_limit * _ctrlComp->dt;
    for (int i = 0; i < 12; ++i) {
        double clamped = clampValue(_tau(i), -f.tau_limit, f.tau_limit);
        double delta = clampValue(clamped - _tauPrev(i), -max_delta, max_delta);
        _tau(i) = clampValue(_tauPrev(i) + delta, -f.tau_limit, f.tau_limit);
    }
    _tauPrev = _tau;
}

void State_FixedStand::exit() {
    for (int leg = 0; leg < 4; ++leg) {
        _lowCmd->setZeroTau(leg);
    }
    _percent = 0.0f;
    _forceControlActive = false;
    _tauPrev.setZero();
}

float State_FixedStand::transitionBlend() const {
    const float s = std::min(std::max(_percent, 0.0f), 1.0f);
    return s * s * (3.0f - 2.0f * s);
}

FSMStateName State_FixedStand::checkChange() {
    if (_lowState->userCmd == UserCommand::L2_B) {
        return FSMStateName::PASSIVE;
    }
    if (_lowState->userCmd == UserCommand::L2_X) {
        return FSMStateName::TROTTING;
    }
    return FSMStateName::FIXEDSTAND;
}

Vec3 State_FixedStand::clampJointAngles(const Vec3& q) const {
    const Vec3 lower = _ctrlComp->parameters.joint_limits.lower();
    const Vec3 upper = _ctrlComp->parameters.joint_limits.upper();
    Vec3 clamped = q;
    for (int i = 0; i < 3; ++i) {
        clamped(i) = std::min(std::max(clamped(i), lower(i)), upper(i));
    }
    return clamped;
}

Vec3 State_FixedStand::computeBaseStandFootTargetInHip(int leg) const {
    return _ctrlComp->parameters.stand_targets.normal_feet_in_hip[leg];
}

Vec3 State_FixedStand::computeCompensatedFootTargetInHip(
    int leg, const Vec3& base_target_in_hip) const {
    const auto& stand = _ctrlComp->parameters.stand;
    Vec3 base_target_in_body = _ctrlComp->parameters.hip_mounts_in_body[leg] + base_target_in_hip;
    Vec3 delta_body = Vec3::Zero();

    const Vec3 rpy = rotMatToRPY(_lowState->getRotMat());
    if (rpy.allFinite() && stand.roll_pitch_comp_gain > 0.0) {
        const RotMat compensation_rotation =
            rpyToRotMat(-stand.roll_pitch_comp_gain * rpy.x(),
                        -stand.roll_pitch_comp_gain * rpy.y(),
                        0.0);
        const Vec3 rotated_target = compensation_rotation * base_target_in_body;
        delta_body += rotated_target - base_target_in_body;
    }

    double nominal_body_height = 0.0;
    for (const Vec3& target_in_hip : _targetFeetInHip) {
        nominal_body_height += -target_in_hip.z();
    }
    nominal_body_height =
        nominal_body_height / static_cast<double>(qr_guide::NumLeg) + _ctrlComp->parameters.foot_radius_m;

    if (_ctrlComp->estimator != nullptr && stand.height_comp_gain > 0.0 && nominal_body_height > 1e-6) {
        const Vec3 estimated_position = _ctrlComp->estimator->getPosition();
        if (estimated_position.allFinite()) {
            const double height_error = nominal_body_height - estimated_position.z();
            delta_body.z() -= clampValue(height_error * stand.height_comp_gain,
                                         -stand.compensation_limit,
                                         stand.compensation_limit);
        }
    }

    const double delta_norm = delta_body.norm();
    if (delta_norm > stand.compensation_limit && delta_norm > 1e-9) {
        delta_body *= stand.compensation_limit / delta_norm;
    }

    Vec3 target_in_body = base_target_in_body + delta_body;
    Vec3 target_in_hip = target_in_body - _ctrlComp->parameters.hip_mounts_in_body[leg];
    target_in_hip.z() = std::min(target_in_hip.z(), -0.05);
    return target_in_hip;
}
