#include "FSM/State_FixedStand.h"
#include "FSM/StateMotorParams.h"

#include <algorithm>
#include <iomanip>
#include <iostream>

namespace {

constexpr int kHybridDebugPrintEveryCycles = 100;
constexpr double kRadToDeg = 57.29577951308232;
constexpr bool kEnableHybridDebugPrint = false;

const char* HybridDebugStatusToString(qr_guide::HybridStandController::DebugStatus status) {
    switch (status) {
    case qr_guide::HybridStandController::DebugStatus::IDLE:
        return "idle";
    case qr_guide::HybridStandController::DebugStatus::SUCCESS:
        return "success";
    case qr_guide::HybridStandController::DebugStatus::MOTOR_FAULT:
        return "motor_fault";
    case qr_guide::HybridStandController::DebugStatus::INVALID_STATE:
        return "invalid_state";
    case qr_guide::HybridStandController::DebugStatus::INVALID_FEET:
        return "invalid_feet";
    case qr_guide::HybridStandController::DebugStatus::ALLOCATION_FAIL:
        return "allocation_fail";
    case qr_guide::HybridStandController::DebugStatus::INVALID_TORQUE:
        return "invalid_torque";
    }
    return "unknown";
}

}  // namespace

State_FixedStand::State_FixedStand(CtrlComponents* ctrlComp)
    : FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand") {
    if (ctrlComp->parameters.hybrid_stand.enabled) {
        _hybridStandController =
            std::make_unique<qr_guide::HybridStandController>(ctrlComp->parameters.hybrid_stand, ctrlComp->dt);
    }
    for (auto& target : _targetFeetInBody) {
        target.setZero();
    }
}

void State_FixedStand::enter() {
    for (int leg = 0; leg < 4; ++leg) {
        fsm_motor_params::ApplyLegProfile(_lowCmd, leg, fsm_motor_params::kFixedStandProfile);
    }

    for (int i = 0; i < 12; ++i) {
        _startPos(i) = _lowState->motorState[i].q;
    }

    _currentMode = NORMAL_STAND;
    const auto& targets = _ctrlComp->parameters.stand_targets.normal_feet_in_hip;

    for (int leg = 0; leg < 4; ++leg) {
        // 固定站立和匍匐都统一走参数文件中的足端目标，不再散落硬编码。
        Vec3 q = _ctrlComp->robotModel->inverseKinematics(targets[leg], leg, FrameType::HIP);
        q = clampJointAngles(q);
        _targetPos.segment(leg * 3, 3) = q;
        _targetFeetInBody[leg] = _ctrlComp->parameters.hip_mounts_in_body[leg] + targets[leg];
    }

    _percent = 0.0f;
    _debugPrintCounter = 0;
    if (_hybridStandController) {
        _hybridStandController->reset();
    }
    _ctrlComp->setAllStance();
}

void State_FixedStand::run() {
    // 从进入状态时的实测关节角平滑插值到目标关节角，避免突变。
    _percent += 1.0f / static_cast<float>(_duration);
    if (_percent > 1.0f) {
        _percent = 1.0f;
    }

    Vec12 q_cmd = Vec12::Zero();
    for (int i = 0; i < 12; ++i) {
        _lowCmd->motorCmd[i].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
        _lowCmd->motorCmd[i].dq = 0.0f;
        q_cmd(i) = (1.0f - _percent) * _startPos(i) + _percent * _targetPos(i);
        _lowCmd->motorCmd[i].q = q_cmd(i);
    }

    Vec12 tau_ff = Vec12::Zero();
    bool hybrid_success = false;
    if (_hybridStandController) {
        hybrid_success = _hybridStandController->computeTorqueFeedforward(
            _targetFeetInBody, *_lowState, _ctrlComp->estimator.get(), *_ctrlComp->robotModel, _percent, &tau_ff);
    }

    if (!hybrid_success) {
        tau_ff.setZero();
        if (_hybridStandController) {
            _hybridStandController->reset();
        }
    }

    for (int i = 0; i < 12; ++i) {
        _lowCmd->motorCmd[i].tau = static_cast<float>(tau_ff(i));
    }

    if (kEnableHybridDebugPrint && _hybridStandController != nullptr &&
        ++_debugPrintCounter >= kHybridDebugPrintEveryCycles) {
        _debugPrintCounter = 0;
        const auto& debug = _hybridStandController->debugSnapshot();
        const auto old_flags = std::cout.flags();
        const auto old_precision = std::cout.precision();
        std::cout << std::fixed << std::setprecision(3)
                  << "[FixedStand][HybridDebug] mode="
                  << ((_currentMode == NORMAL_STAND) ? "normal" : "crouch")
                  << " status=" << HybridDebugStatusToString(debug.status)
                  << " transition=" << _percent
                  << " roll_deg=" << debug.rpy.x() * kRadToDeg
                  << " pitch_deg=" << debug.rpy.y() * kRadToDeg
                  << " z=" << debug.position.z()
                  << " z_des=" << debug.z_des
                  << " wx=" << debug.gyro.x()
                  << " wy=" << debug.gyro.y()
                  << " Fz=" << debug.body_task_wrench.x()
                  << " Mx=" << debug.body_task_wrench.y()
                  << " My=" << debug.body_task_wrench.z()
                  << " fz[FR,FL,RR,RL]=["
                  << debug.vertical_forces(qr_guide::FR) << ","
                  << debug.vertical_forces(qr_guide::FL) << ","
                  << debug.vertical_forces(qr_guide::RR) << ","
                  << debug.vertical_forces(qr_guide::RL) << "]"
                  << std::endl;
        std::cout.flags(old_flags);
        std::cout.precision(old_precision);
    }
}

void State_FixedStand::exit() {
    for (int leg = 0; leg < 4; ++leg) {
        _lowCmd->setZeroTau(leg);
    }
    if (_hybridStandController) {
        _hybridStandController->reset();
    }
    _percent = 0.0f;
    _debugPrintCounter = 0;
    _currentMode = NORMAL_STAND;
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
    // IK 结果在这里做一次机械限位保护。
    const Vec3 lower = _ctrlComp->parameters.joint_limits.lower();
    const Vec3 upper = _ctrlComp->parameters.joint_limits.upper();
    Vec3 clamped = q;
    for (int i = 0; i < 3; ++i) {
        clamped(i) = std::min(std::max(clamped(i), lower(i)), upper(i));
    }
    return clamped;
}
