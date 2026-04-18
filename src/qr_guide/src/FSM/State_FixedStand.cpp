#include "FSM/State_FixedStand.h"
#include "FSM/StateMotorParams.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace {

template<typename T>
T clampValue(T value, T min_value, T max_value) {
    return std::min(std::max(value, min_value), max_value);
}

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

    _currentMode = NORMAL_STAND;
    _duration = std::max(1, _ctrlComp->parameters.stand.entry_duration);

    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 q = _lowState->getQ().col(leg);
        _startFeetInHip[leg] = _ctrlComp->robotModel->forwardKinematics(q, leg, FrameType::HIP);
        _targetFeetInHip[leg] = computeBaseStandFootTargetInHip(leg);
        _targetFeetInBody[leg] =
            _ctrlComp->parameters.hip_mounts_in_body[leg] + _targetFeetInHip[leg];
    }

    _percent = 0.0f;
    _debugPrintCounter = 0;
    if (_hybridStandController) {
        _hybridStandController->reset();
    }
    _ctrlComp->setAllStance();
}

void State_FixedStand::run() {
    _percent += 1.0f / static_cast<float>(_duration);
    if (_percent > 1.0f) {
        _percent = 1.0f;
    }

    const float blend = transitionBlend();
    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 base_target =
            (1.0f - blend) * _startFeetInHip[leg] + blend * _targetFeetInHip[leg];
        const Vec3 compensated_target = computeCompensatedFootTargetInHip(leg, base_target);
        _targetFeetInBody[leg] =
            _ctrlComp->parameters.hip_mounts_in_body[leg] + compensated_target;

        Vec3 q = _ctrlComp->robotModel->inverseKinematics(compensated_target, leg, FrameType::HIP);
        q = clampJointAngles(q);

        for (int joint = 0; joint < 3; ++joint) {
            const int id = leg * 3 + joint;
            _lowCmd->motorCmd[id].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            _lowCmd->motorCmd[id].dq = 0.0f;
            _lowCmd->motorCmd[id].q = q(joint);
            _lowCmd->motorCmd[id].tau = 0.0f;
        }
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

Vec3 State_FixedStand::computeCompensatedFootTargetInHip(int leg, const Vec3& base_target_in_hip) const {
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
