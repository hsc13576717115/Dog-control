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
    _shouldEmergencyStop = false;
    _forceRampPercent = 0.0;
    _handoffWaitPrintCounter = 0;
    _tauPrev.setZero();

    std::cout << "\n========== [FixedStand] ENTER ==========" << std::endl;

    for (int leg = 0; leg < 4; ++leg) {
        fsm_motor_params::ApplyLegProfile(_lowCmd, leg, fsm_motor_params::kFixedStandProfile);

        const Vec3 q = _lowState->getQ().col(leg);
        _startFeetInHip[leg] = _robModel->forwardKinematics(q, leg, FrameType::HIP);

        // VMC 站立保留配置里的 x/z：x=-0.03 用来把膝盖带离地面；
        // y 保留进入瞬间的腿宽，避免 12 自由度髋外展通道在起身时内收。
        Vec3 base_target = computeBaseStandFootTargetInHip(leg);
        base_target.y() = _startFeetInHip[leg].y();
        _targetFeetInHip[leg] = base_target;

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

    // 用运动学计算当前真实高度（绕开 estimator IMU 漂移）
    double current_height = 0.0;
    for (int leg = 0; leg < 4; ++leg) {
        Vec3 foot_in_body = _robModel->forwardKinematics(_lowState->getQ().col(leg), leg, FrameType::BODY);
        current_height += -foot_in_body(2);
    }
    current_height /= 4.0;
    current_height += _ctrlComp->parameters.foot_radius_m;

    // 初始化站立期望状态：x/y 用 estimator，z 用运动学真实高度
    _pcd = _est->getPosition();
    _standTargetHeight = current_height;
    _currentTargetHeight = current_height;
    _pcd(2) = _currentTargetHeight;

    // 记录初始 yaw，期望保持水平姿态（roll=0, pitch=0, yaw=初始yaw）
    const Vec3 init_rpy = rotMatToRPY(_lowState->getRotMat());
    _Rd = rpyToRotMat(0.0, 0.0, init_rpy(2));

    const Vec3 rpy = rotMatToRPY(_lowState->getRotMat());
    std::cout << "[FixedStand] init pos=" << _pcd.transpose()
              << " rpy=" << rpy.transpose()
              << " current_height=" << current_height
              << " body_height=" << _ctrlComp->parameters.stand.body_height
              << " hybrid_force="
              << (_ctrlComp->parameters.hybrid_stand.enabled ? "enabled" : "disabled")
              << " force_mode=" << _ctrlComp->parameters.hybrid_stand.force_mode
              << " max_force_scale=" << _ctrlComp->parameters.hybrid_stand.max_force_scale
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

    // 显式打开 hybrid_stand.enabled 后，先用位置环把腿摆到站立几何姿态，
    // 再让 VMC 力矩从 0 爬升接管支撑，避免趴地姿态下纯力矩直接顶地。
    if (!_ctrlComp->parameters.hybrid_stand.enabled) {
        runPositionTransition(false, false);
        // 力控状态清零，避免残留
        resetForceControlState();
        return;
    }

    if (_percent < 1.0f) {
        runPositionTransition(true, false);
        resetForceControlState();
        return;
    }

    if (!_forceControlActive) {
        double actual_height = 0.0;
        Vec3 rpy = Vec3::Zero();
        Vec3 max_foot_error = Vec3::Zero();
        if (!isVmcHandoffReady(&actual_height, &rpy, &max_foot_error)) {
            runPositionTransition(true, false);
            resetForceControlState();
            if (++_handoffWaitPrintCounter % 50 == 0) {
                const auto& hybrid = _ctrlComp->parameters.hybrid_stand;
                std::cout << std::fixed << std::setprecision(3)
                          << "[FixedStand][handoff_wait]"
                          << " height=" << actual_height
                          << "/" << hybrid.vmc_handoff_min_height_m
                          << " rpy=" << rpy.transpose()
                          << " maxFootErr=" << max_foot_error.transpose()
                          << " limit=" << hybrid.vmc_handoff_max_foot_error_m.transpose()
                          << " posKp=" << hybrid.vmc_prehandoff_joint_kp.transpose()
                          << " posKd=" << hybrid.vmc_prehandoff_joint_kd.transpose()
                          << std::endl;
            }
            return;
        }
        _handoffWaitPrintCounter = 0;
        std::cout << std::fixed << std::setprecision(3)
                  << "[FixedStand] handoff ready"
                  << " height=" << actual_height
                  << " rpy=" << rpy.transpose()
                  << " maxFootErr=" << max_foot_error.transpose()
                  << std::endl;
    }

    runForceControl(1.0f);
}

void State_FixedStand::runPositionTransition(bool use_handoff_gains, bool lift_assist) {
    Vec12 assist_tau = Vec12::Zero();
    const auto& hybrid = _ctrlComp->parameters.hybrid_stand;
    const double assist_scale = lift_assist ? computePreHandoffLiftScale() : 0.0;
    const double front_rear_height_diff = computeFrontRearHeightDiff();
    const double sync_z = lift_assist
        ? clampValue(hybrid.vmc_lift_sync_z_gain * front_rear_height_diff,
                     -std::abs(hybrid.vmc_lift_sync_z_limit_m),
                     std::abs(hybrid.vmc_lift_sync_z_limit_m))
        : 0.0;
    if (assist_scale > 1e-6) {
        Vec34 assist_force_body = Vec34::Zero();
        const double gravity_per_leg = _robModel->getRobMass() * 9.81 / 4.0;
        const double fz_limit = std::max(0.0, hybrid.fz_max_per_leg_n);
        const double assist_fz = std::min(gravity_per_leg * assist_scale, fz_limit);
        const double pitch_load_shift =
            computePitchLoadShift(rotMatToRPY(_lowState->getRotMat()),
                                  _lowState->getGyro(),
                                  static_cast<double>(_percent)) *
            clampValue(assist_scale, 0.0, 1.0);
        const double sync_load_shift =
            computeLiftSyncLoadShift(front_rear_height_diff, static_cast<double>(_percent)) *
            clampValue(assist_scale, 0.0, 1.0);
        const double total_load_shift = pitch_load_shift + sync_load_shift;
        for (int leg = 0; leg < 4; ++leg) {
            const bool front_leg = (leg == 0 || leg == 1);
            const double load_delta = front_leg ? -total_load_shift : total_load_shift;
            assist_force_body(2, leg) = clampValue(assist_fz + load_delta, 0.0, fz_limit);
        }

        assist_tau = -_robModel->getTau(vec34ToVec12(_lowState->getQ()), assist_force_body);
        const double assist_tau_limit = std::min(
            std::abs(hybrid.tau_limit_nm),
            std::abs(hybrid.vmc_prehandoff_tau_limit_nm));
        for (int leg = 0; leg < 4; ++leg) {
            assist_tau(leg * 3) = 0.0;  // handoff 前 q0 完全交给位置环保持腿宽
        }
        for (int i = 0; i < 12; ++i) {
            assist_tau(i) = clampValue(assist_tau(i), -assist_tau_limit, assist_tau_limit);
        }
    }

    for (int leg = 0; leg < 4; ++leg) {
        Vec3 base_target =
            (1.0f - _percent) * _startFeetInHip[leg] + _percent * _targetFeetInHip[leg];
        if (lift_assist) {
            const bool front_leg = (leg == 0 || leg == 1);
            base_target.z() += front_leg ? sync_z : -sync_z;
        }
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
        fsm_motor_params::ApplyLegProfile(_lowCmd, leg, fsm_motor_params::kFixedStandProfile);
        for (int joint = 0; joint < 3; ++joint) {
            const int id = leg * 3 + joint;
            if (use_handoff_gains) {
                _lowCmd->motorCmd[id].Kp =
                    static_cast<float>(std::max(0.0, hybrid.vmc_prehandoff_joint_kp(joint)));
                _lowCmd->motorCmd[id].Kd =
                    static_cast<float>(std::max(0.0, hybrid.vmc_prehandoff_joint_kd(joint)));
            }
            _lowCmd->motorCmd[id].tau = assist_tau(id);
        }
    }

    static int pos_dbg_cnt = 0;
    if (use_handoff_gains && ++pos_dbg_cnt % 50 == 0) {
        std::cout << std::fixed << std::setprecision(3)
                  << "[FixedStand][pos_only]"
                  << " percent=" << _percent
                  << " height=" << computeActualBodyHeight()
                  << " kp=" << hybrid.vmc_prehandoff_joint_kp.transpose()
                  << " kd=" << hybrid.vmc_prehandoff_joint_kd.transpose()
                  << " frHeightDiff=" << front_rear_height_diff
                  << " tau_FR=" << assist_tau.segment(0, 3).transpose()
                  << " tau_FL=" << assist_tau.segment(3, 3).transpose()
                  << std::endl;
    }
}

void State_FixedStand::runForceControl(float blend) {
    const auto& hybrid = _ctrlComp->parameters.hybrid_stand;
    if (!_forceControlActive) {
        _forceControlActive = true;
        _forceRampPercent = 0.0;
        _tauPrev.setZero();
        std::cout << "[FixedStand] position transition complete, switching to hybrid VMC"
                  << std::endl;
    }

    const double ramp_duration = std::max(0.1, hybrid.force_ramp_duration_s);
    _forceRampPercent = clampValue(
        _forceRampPercent + _ctrlComp->dt / ramp_duration, 0.0, 1.0);
    const double force_scale =
        clampValue(hybrid.max_force_scale, 0.0, 1.0) * _forceRampPercent;

    // 1. 获取机身状态
    Vec3 posBody = _est->getPosition();
    Vec3 velBody = _est->getVelocity();

    // 用运动学计算真实高度，绕开 estimator IMU 漂移
    const double actual_height = computeActualBodyHeight();
    posBody(2) = actual_height;  // 用运动学高度覆盖 estimator 漂移值
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    const Vec3 cur_rpy = rotMatToRPY(_B2G_RotMat);

    // 2. 目标：保持当前 x,y 水平位置（避免水平恢复力导致足端滑动），固定高度，水平姿态
    _pcd(0) = posBody(0);
    _pcd(1) = posBody(1);
    _pcd(2) = _ctrlComp->parameters.stand.body_height;

    const Vec3 posError = _pcd - posBody;
    const Vec3 velError = -velBody;

    _ddPcd = _Kpp * posError + _Kdp * velError;
    Vec3 rot_error = rotMatToExp(_Rd * _G2B_RotMat);
    rot_error(2) = 0.0;  // yaw 不控制
    // VMC 逐腿弹簧模式只保留陀螺仪阻尼，roll/pitch 用下方竖直力差分补偿。
    _dWbd = _Kdw * (-_lowState->getGyroGlobal());

    // dWbd 低通滤波，减少力矩抖动（alpha=0.3，截止频率约 50Hz）
    static Vec3 dWbd_filtered = Vec3::Zero();
    dWbd_filtered = 0.7 * dWbd_filtered + 0.3 * _dWbd;
    _dWbd = dWbd_filtered;

    const auto& f = _ctrlComp->parameters.force;
    _ddPcd(0) = saturation(_ddPcd(0), f.acc_xy_sat);
    _ddPcd(1) = saturation(_ddPcd(1), f.acc_xy_sat);
    _ddPcd(2) = saturation(_ddPcd(2), f.acc_z_sat);
    _dWbd(0) = saturation(_dWbd(0), f.w_roll_pitch_sat);
    _dWbd(1) = saturation(_dWbd(1), f.w_roll_pitch_sat);
    _dWbd(2) = saturation(_dWbd(2), f.w_yaw_sat);

    // 3. VMC 足端虚拟弹簧/阻尼。足端力定义为地面对机身的等效支撑力，hip/body 方向一致。
    Vec12 q = vec34ToVec12(_lowState->getQ());
    Vec34 foot_errors = Vec34::Zero();
    Vec34 foot_velocities = Vec34::Zero();
    _forceFeetBody.setZero();
    const double gravity_per_leg =
        _robModel->getRobMass() * 9.81 * clampValue(hybrid.vmc_gravity_scale, 0.0, 2.0) / 4.0;
    const double fz_min = std::min(hybrid.fz_min_per_leg_n, hybrid.fz_max_per_leg_n);
    const double fz_max = std::max(hybrid.fz_min_per_leg_n, hybrid.fz_max_per_leg_n);
    const double friction_ratio = std::max(0.0, _ctrlComp->parameters.force.friction_ratio);
    const Vec3 gyro_body = _lowState->getGyro();
    const double attitude_blend = clampValue((static_cast<double>(blend) - 0.12) / 0.55, 0.0, 1.0);
    const double roll_torque_cmd =
        attitude_blend * (hybrid.kp_roll * (0.0 - cur_rpy.x()) +
                          hybrid.kd_roll * (0.0 - gyro_body.x()));
    const double pitch_torque_cmd =
        attitude_blend * (hybrid.kp_pitch * (0.0 - cur_rpy.y()) +
                          hybrid.kd_pitch * (0.0 - gyro_body.y()));
    double half_length = 0.0;
    double half_track = 0.0;
    for (const Vec3& hip : _ctrlComp->parameters.hip_mounts_in_body) {
        half_length += std::abs(hip.x());
        half_track += std::abs(hip.y());
    }
    half_length = std::max(0.01, half_length / 4.0);
    half_track = std::max(0.01, half_track / 4.0);
    const double roll_force_unit = -roll_torque_cmd / (4.0 * half_track);
    const double pitch_force_unit = -pitch_torque_cmd / (4.0 * half_length);
    const double attitude_fz_limit = clampValue((fz_max - fz_min) * 0.10, 2.0, 6.0);
    const double pitch_load_shift = computePitchLoadShift(cur_rpy, gyro_body, blend);
    const double front_rear_height_diff = computeFrontRearHeightDiff();
    const double sync_load_shift = computeLiftSyncLoadShift(front_rear_height_diff, blend);
    const double front_rear_load_shift = pitch_load_shift + sync_load_shift;
    Vec4 attitude_fz = Vec4::Zero();
    Vec4 front_rear_load_fz = Vec4::Zero();

    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 q_leg = _lowState->getQ().col(leg);
        const Vec3 foot_now = _robModel->forwardKinematics(q_leg, leg, FrameType::HIP);
        const Vec3 foot_target =
            (1.0f - blend) * _startFeetInHip[leg] + blend * _targetFeetInHip[leg];
        const Vec3 foot_vel = _robModel->getFootVelocity(*_lowState, leg);

        Vec3 foot_error = foot_target - foot_now;
        for (int axis = 0; axis < 3; ++axis) {
            const double limit = std::abs(hybrid.vmc_error_limit_m(axis));
            foot_error(axis) = clampValue(foot_error(axis), -limit, limit);
        }

        foot_errors.col(leg) = foot_error;
        foot_velocities.col(leg) = foot_vel;
    }

    for (int leg = 0; leg < 4; ++leg) {
        Vec3 foot_force =
            -hybrid.vmc_kp_foot.cwiseProduct(foot_errors.col(leg)) +
            hybrid.vmc_kd_foot.cwiseProduct(foot_velocities.col(leg));
        foot_force.z() += gravity_per_leg;
        const bool right_leg = (leg == 0 || leg == 2);
        const bool front_leg = (leg == 0 || leg == 1);
        const double roll_delta = right_leg ? roll_force_unit : -roll_force_unit;
        const double pitch_delta = front_leg ? pitch_force_unit : -pitch_force_unit;
        front_rear_load_fz(leg) = front_leg ? -front_rear_load_shift : front_rear_load_shift;
        attitude_fz(leg) = clampValue(roll_delta + pitch_delta,
                                      -attitude_fz_limit,
                                      attitude_fz_limit);
        foot_force.z() += attitude_fz(leg);
        foot_force.z() += front_rear_load_fz(leg);
        foot_force.z() = clampValue(foot_force.z(), fz_min, fz_max);

        const double fxy_limit = friction_ratio * std::max(foot_force.z(), 0.0);
        foot_force.x() = clampValue(foot_force.x(), -fxy_limit, fxy_limit);
        foot_force.y() = clampValue(foot_force.y(), -fxy_limit, fxy_limit);

        _forceFeetBody.col(leg) = foot_force;
    }
    _forceFeetGlobal = _B2G_RotMat * _forceFeetBody;

    // 4. Jacobian 转置映射力矩（符号沿用实机已验证的 QP 力矩链路）
    _tau = -_robModel->getTau(q, _forceFeetBody);

    // 5. FixedStand 试验模式：位置环托底，VMC 力控前馈按配置比例缓慢注入。
    _tau *= force_scale;

    // 8 自由度参考 VMC 没有髋外展通道。12 自由度实机上，竖直支撑力经 3D
    // Jacobian 会天然映射出 q0 力矩，容易在起身阶段把足端往内收；FixedStand
    // 对 q0 只保留很小的前馈力矩，并在发命令时用软位置环保持腿宽。
    const double q0_tau_limit_nm = std::abs(hybrid.vmc_q0_tau_limit_nm);
    for (int leg = 0; leg < 4; ++leg) {
        _tau(leg * 3) = clampValue(_tau(leg * 3), -q0_tau_limit_nm, q0_tau_limit_nm);
    }

    // 6. 力矩安全限幅
    applyTorqueSafety();

    // 7. 姿态安全阈值：|roll| 或 |pitch| 超过 15° 触发保护
    if (std::fabs(cur_rpy(0)) > 0.26 || std::fabs(cur_rpy(1)) > 0.26) {
        std::cerr << "[FixedStand][WARN] 姿态超限 roll=" << cur_rpy(0)
                  << " pitch=" << cur_rpy(1) << "，请求急停" << std::endl;
        _shouldEmergencyStop = true;
    }

    // 8. 输出复合命令：VMC 力控前馈叠加低刚度位置保持，限制足端漂移。
    const double max_force_scale = clampValue(hybrid.max_force_scale, 0.0, 1.0);
    const double force_gain_blend =
        (max_force_scale > 1e-6) ? clampValue(force_scale / max_force_scale, 0.0, 1.0) : 1.0;
    const Vec3 handoff_kp = hybrid.vmc_prehandoff_joint_kp.cwiseMax(Vec3::Zero());
    const Vec3 handoff_kd = hybrid.vmc_prehandoff_joint_kd.cwiseMax(Vec3::Zero());
    const Vec3 hold_kp = hybrid.vmc_joint_hold_kp.cwiseMax(Vec3::Zero());
    const Vec3 hold_kd = hybrid.vmc_joint_hold_kd.cwiseMax(Vec3::Zero());
    const Vec3 force_phase_kp =
        (1.0 - force_gain_blend) * handoff_kp +
        force_gain_blend * hold_kp;
    const Vec3 force_phase_kd =
        (1.0 - force_gain_blend) * handoff_kd +
        force_gain_blend * hold_kd;
    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 compensated_target =
            (1.0f - blend) * _startFeetInHip[leg] + blend * _targetFeetInHip[leg];
        Vec3 q = _robModel->inverseKinematics(compensated_target, leg, FrameType::HIP);
        q = clampJointAngles(q);

        for (int joint = 0; joint < 3; ++joint) {
            const int id = leg * 3 + joint;
            _lowCmd->motorCmd[id].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            _lowCmd->motorCmd[id].dq = 0.0f;
            _lowCmd->motorCmd[id].q = q(joint);
            _lowCmd->motorCmd[id].Kp = static_cast<float>(force_phase_kp(joint));
            _lowCmd->motorCmd[id].Kd = static_cast<float>(force_phase_kd(joint));
            _lowCmd->motorCmd[id].tau = _tau(id);
        }
    }

    // ===== 力控调试输出（每 50 帧 ≈ 0.1s 打印一次）=====
    static int dbg_cnt = 0;
    if (++dbg_cnt % 50 == 0) {
        const double total_fz = _forceFeetBody.row(2).sum();
        const double gravity = _robModel->getRobMass() * 9.81;
        std::cout << std::fixed << std::setprecision(3)
                  << "[FixedStand][dbg] mode=" << hybrid.force_mode
                  << " blend=" << blend
                  << " forceScale=" << force_scale
                  << " softBlend=" << force_gain_blend
                  << " q0Kp=" << force_phase_kp(0)
                  << " q0Kd=" << force_phase_kd(0)
                  << " q12Kp=" << force_phase_kp(1) << "," << force_phase_kp(2)
                  << " q12Kd=" << force_phase_kd(1) << "," << force_phase_kd(2)
                  << " footErr_FR=" << foot_errors.col(0).transpose()
                  << " footVel_FR=" << foot_velocities.col(0).transpose()
                  << " attFz=" << attitude_fz.transpose()
                  << " frHeightDiff=" << front_rear_height_diff
                  << " pitchLoad=" << pitch_load_shift
                  << " syncLoad=" << sync_load_shift
                  << " frontRearFz=" << front_rear_load_fz.transpose()
                  << " posErr=" << posError.transpose()
                  << " rpy=" << cur_rpy.transpose()
                  << " ddPcd=" << _ddPcd.transpose()
                  << " dWbd=" << _dWbd.transpose()
                  << " Fz_sum=" << total_fz << "/" << gravity
                  << " Fz_cmd=" << total_fz * force_scale
                  << " tau_FR=" << _tau.segment(0,3).transpose()
                  << " tau_FL=" << _tau.segment(3,3).transpose()
                  << std::endl;
    }
}

void State_FixedStand::applyTorqueSafety() {
    const auto& hybrid = _ctrlComp->parameters.hybrid_stand;
    const double tau_limit = std::abs(hybrid.tau_limit_nm);
    const double max_delta = std::abs(hybrid.tau_rate_limit_nm_per_s) * _ctrlComp->dt;
    for (int i = 0; i < 12; ++i) {
        double clamped = clampValue(_tau(i), -tau_limit, tau_limit);
        double delta = clampValue(clamped - _tauPrev(i), -max_delta, max_delta);
        _tau(i) = clampValue(_tauPrev(i) + delta, -tau_limit, tau_limit);
    }
    _tauPrev = _tau;
}

double State_FixedStand::computeActualBodyHeight() const {
    double actual_height = 0.0;
    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 foot_in_body =
            _robModel->forwardKinematics(_lowState->getQ().col(leg), leg, FrameType::BODY);
        actual_height += -foot_in_body(2);
    }
    actual_height /= static_cast<double>(qr_guide::NumLeg);
    actual_height += _ctrlComp->parameters.foot_radius_m;
    return actual_height;
}

Vec3 State_FixedStand::computeMaxAbsFootError(float blend) const {
    Vec3 max_error = Vec3::Zero();
    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 foot_now =
            _robModel->forwardKinematics(_lowState->getQ().col(leg), leg, FrameType::HIP);
        const Vec3 foot_target =
            (1.0f - blend) * _startFeetInHip[leg] + blend * _targetFeetInHip[leg];
        max_error = max_error.cwiseMax((foot_target - foot_now).cwiseAbs());
    }
    return max_error;
}

bool State_FixedStand::isVmcHandoffReady(
    double* actual_height, Vec3* rpy, Vec3* max_foot_error) const {
    const auto& hybrid = _ctrlComp->parameters.hybrid_stand;
    const double height = computeActualBodyHeight();
    const Vec3 current_rpy = rotMatToRPY(_lowState->getRotMat());
    const Vec3 foot_error = computeMaxAbsFootError(1.0f);

    if (actual_height != nullptr) {
        *actual_height = height;
    }
    if (rpy != nullptr) {
        *rpy = current_rpy;
    }
    if (max_foot_error != nullptr) {
        *max_foot_error = foot_error;
    }

    const double min_height = std::max(0.0, hybrid.vmc_handoff_min_height_m);
    const double max_rp = std::abs(hybrid.vmc_handoff_max_rp_rad);
    const Vec3 foot_error_limit = hybrid.vmc_handoff_max_foot_error_m.cwiseAbs();

    if (height < min_height) {
        return false;
    }
    if (std::fabs(current_rpy(0)) > max_rp || std::fabs(current_rpy(1)) > max_rp) {
        return false;
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (foot_error(axis) > foot_error_limit(axis)) {
            return false;
        }
    }
    return true;
}

double State_FixedStand::computePreHandoffLiftScale() const {
    const auto& hybrid = _ctrlComp->parameters.hybrid_stand;
    const double max_lift_scale =
        clampValue(hybrid.vmc_prehandoff_lift_scale, 0.0, 1.5);
    if (max_lift_scale <= 1e-6) {
        return 0.0;
    }

    // 位置目标先移动一点再加力，避免刚进入 FixedStand 的瞬间顶地太猛。
    const double ramp = clampValue((static_cast<double>(_percent) - 0.08) / 0.72,
                                   0.0,
                                   1.0);
    return max_lift_scale * ramp;
}

double State_FixedStand::computePitchLoadShift(
    const Vec3& rpy, const Vec3& gyro_body, double blend) const {
    const auto& hybrid = _ctrlComp->parameters.hybrid_stand;
    const double shift_limit = std::abs(hybrid.vmc_pitch_load_shift_limit_n);
    if (shift_limit <= 1e-6) {
        return 0.0;
    }

    const double active_blend = clampValue((blend - 0.08) / 0.45, 0.0, 1.0);
    const double raw_shift =
        -hybrid.vmc_pitch_load_shift_kp * rpy.y() -
        hybrid.vmc_pitch_load_shift_kd * gyro_body.y();
    return active_blend * clampValue(raw_shift, -shift_limit, shift_limit);
}

double State_FixedStand::computeFrontRearHeightDiff() const {
    double front_height = 0.0;
    double rear_height = 0.0;
    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 foot_in_body =
            _robModel->forwardKinematics(_lowState->getQ().col(leg), leg, FrameType::BODY);
        const double leg_height = -foot_in_body.z() + _ctrlComp->parameters.foot_radius_m;
        if (leg == 0 || leg == 1) {
            front_height += leg_height;
        } else {
            rear_height += leg_height;
        }
    }
    front_height *= 0.5;
    rear_height *= 0.5;
    return front_height - rear_height;
}

double State_FixedStand::computeLiftSyncLoadShift(
    double front_rear_height_diff, double blend) const {
    const auto& hybrid = _ctrlComp->parameters.hybrid_stand;
    const double shift_limit = std::abs(hybrid.vmc_lift_sync_load_limit_n);
    if (shift_limit <= 1e-6) {
        return 0.0;
    }

    const double active_blend = clampValue((blend - 0.05) / 0.45, 0.0, 1.0);
    const double raw_shift = hybrid.vmc_lift_sync_load_kp * front_rear_height_diff;
    return active_blend * clampValue(raw_shift, -shift_limit, shift_limit);
}

void State_FixedStand::resetForceControlState() {
    _forceControlActive = false;
    _forceRampPercent = 0.0;
    _tauPrev.setZero();
}

void State_FixedStand::exit() {
    for (int leg = 0; leg < 4; ++leg) {
        _lowCmd->setZeroTau(leg);
    }
    _percent = 0.0f;
    _handoffWaitPrintCounter = 0;
    resetForceControlState();
}

float State_FixedStand::transitionBlend() const {
    const float s = std::min(std::max(_percent, 0.0f), 1.0f);
    return s * s * (3.0f - 2.0f * s);
}

FSMStateName State_FixedStand::checkChange() {
    if (_shouldEmergencyStop || _lowState->userCmd == UserCommand::L2_B) {
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
    Vec3 target = _ctrlComp->parameters.stand_targets.normal_feet_in_hip[leg];
    const double body_height = std::max(_ctrlComp->parameters.stand.body_height, 0.05);
    target.z() = _ctrlComp->parameters.foot_radius_m - body_height;
    target.z() = std::min(target.z(), -0.05);
    return target;
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

    // 禁用 estimator 高度补偿：estimator 在 passive 阶段漂移严重，
    // 用漂移值补偿会把身体抬到错误高度
    // if (_ctrlComp->estimator != nullptr && stand.height_comp_gain > 0.0 ...) ...

    const double delta_norm = delta_body.norm();
    if (delta_norm > stand.compensation_limit && delta_norm > 1e-9) {
        delta_body *= stand.compensation_limit / delta_norm;
    }

    Vec3 target_in_body = base_target_in_body + delta_body;
    Vec3 target_in_hip = target_in_body - _ctrlComp->parameters.hip_mounts_in_body[leg];
    target_in_hip.z() = std::min(target_in_hip.z(), -0.05);
    return target_in_hip;
}
