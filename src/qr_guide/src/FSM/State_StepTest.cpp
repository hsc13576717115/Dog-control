#include "FSM/State_StepTest.h"
#include "FSM/StateMotorParams.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {

constexpr double K_SQUAT_DEPTH = 0.15;
constexpr double K_JUMP_HEIGHT = 0.14;
constexpr double K_MAX_X_FORWARD = 0.30;
constexpr double K_LANDING_OFFSET = 0.08;

double getTimeSec() {
    using namespace std::chrono;
    static const steady_clock::time_point start = steady_clock::now();
    return duration<double>(steady_clock::now() - start).count();
}

Vec3 jumpTraj(double t_cycle) {
    Vec3 relative = Vec3::Zero();

    // 按时间片段生成：下蹲 -> 蹬伸 -> 腾空 -> 落地 -> 回位。
    if (t_cycle < 3.00) {
        const double progress = t_cycle / 3.00;
        relative.z() = K_SQUAT_DEPTH * (1.0 - std::cos(progress * M_PI)) * 0.5;
        relative.x() = 0.05 * std::sin(progress * M_PI * 0.5);
    } else if (t_cycle < 3.20) {
        const double progress = (t_cycle - 3.00) / 0.20;
        relative.z() =
            K_SQUAT_DEPTH - (K_SQUAT_DEPTH + K_JUMP_HEIGHT) * std::pow(progress, 1.2);
        relative.x() = -K_MAX_X_FORWARD * std::pow(progress, 1.3);
    } else if (t_cycle < 3.40) {
        const double progress = (t_cycle - 3.20) / 0.20;
        relative.x() = K_LANDING_OFFSET * progress;
        relative.z() = K_JUMP_HEIGHT;
    } else {
        const double progress = (t_cycle - 3.40) / 0.30;
        if (progress < 0.6) {
            relative.x() = K_LANDING_OFFSET;
            relative.z() = K_JUMP_HEIGHT * (1.0 - progress / 0.6);
        } else {
            const double return_progress = (progress - 0.6) / 0.4;
            relative.x() = K_LANDING_OFFSET * (1.0 - return_progress);
        }
    }

    return relative;
}

}  // namespace

State_StepTest::State_StepTest(CtrlComponents* ctrlComp)
    : FSMState(ctrlComp, FSMStateName::STEPTEST, "step test") {
    for (auto& foot : _initFootPos) {
        foot.setZero();
    }
    for (auto& q : _lastLegQ) {
        q.setZero();
    }
}

void State_StepTest::enter() {
    for (int i = 0; i < 12; ++i) {
        _initMotorQ(i) = _lowState->motorState[i].q;
    }

    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 q = _initMotorQ.segment(leg * 3, 3);
        // 动作起点以进入状态瞬间的实际腿形为准。
        _lastLegQ[leg] = q;
        _initFootPos[leg] = _ctrlComp->robotModel->forwardKinematics(q, leg, FrameType::HIP);
    }

    for (int leg = 0; leg < 4; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            const int id = leg * 3 + joint;
            _lowCmd->motorCmd[id].mode = fsm_motor_params::kCompoundMode;
            _lowCmd->motorCmd[id].dq = 0.0f;
            fsm_motor_params::ApplyGainSet(
                &_lowCmd->motorCmd[id], fsm_motor_params::kStepTestSquatGains);
            _lowCmd->motorCmd[id].q = _lastLegQ[leg](joint);
        }
    }

    _transitionCount = 0;
    _isJumpCompleted = false;
    _isCycleEnded = false;
    _startTime = getTimeSec();
    _ctrlComp->setAllStance();
}

void State_StepTest::run() {
    const double t_total = getTimeSec() - _startTime;
    const double t_cycle = std::fmod(t_total, JUMP_CYCLE_T);
    ++_transitionCount;

    if (t_total > JUMP_CYCLE_T && !_isCycleEnded) {
        _isCycleEnded = true;
        _isJumpCompleted = true;
    }

    fsm_motor_params::GainSet gains = fsm_motor_params::kStepTestReturnGains;
    // 不同阶段使用不同的增益和力矩前馈，尽量兼顾稳定性和动作感。
    if (t_cycle < 3.00) {
        gains = fsm_motor_params::kStepTestSquatGains;
    } else if (t_cycle < 3.20) {
        gains = fsm_motor_params::kStepTestThrustGains;
    } else if (t_cycle < 3.40) {
        gains = fsm_motor_params::kStepTestAirGains;
    } else if (t_cycle < 3.55) {
        gains = fsm_motor_params::kStepTestLandGains;
    }

    const double transition_scale =
        std::min(1.0, static_cast<double>(_transitionCount) / static_cast<double>(TRANSITION_DURATION));

    for (int leg = 0; leg < 4; ++leg) {
        if (_isCycleEnded) {
            continue;
        }

        const Vec3 foot_target = _initFootPos[leg] + jumpTraj(t_cycle) * transition_scale;
        Vec3 q = _ctrlComp->robotModel->inverseKinematics(foot_target, leg, FrameType::HIP);
        // 当前版本将髋关节固定到 0，主要测试矢状面上的动作。
        q(0) = HIP_JOINT_FIXED;
        q = clampJointAngles(q);

        Vec3 delta = q - _lastLegQ[leg];
        const double max_delta = 0.045;
        if (delta.norm() > max_delta) {
            delta = max_delta * delta.normalized();
        }

        const Vec3 q_cmd = _lastLegQ[leg] + delta;
        _lastLegQ[leg] = q_cmd;

        for (int joint = 0; joint < 3; ++joint) {
            const int id = leg * 3 + joint;
            _lowCmd->motorCmd[id].q = q_cmd(joint);
            fsm_motor_params::ApplyGainSet(&_lowCmd->motorCmd[id], gains);
            _lowCmd->motorCmd[id].mode = fsm_motor_params::kCompoundMode;
        }
    }
}

void State_StepTest::exit() {
    for (int leg = 0; leg < 4; ++leg) {
        const int base = leg * 3;
        for (int joint = 0; joint < 3; ++joint) {
            const int id = base + joint;
            _lowCmd->motorCmd[id].q = _initMotorQ(id);
            fsm_motor_params::ApplyGainSet(
                &_lowCmd->motorCmd[id], fsm_motor_params::kStepTestExitGains);
            _lowCmd->motorCmd[id].mode = fsm_motor_params::kCompoundMode;
        }
        _lowCmd->motorCmd[base].q = HIP_JOINT_FIXED;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

FSMStateName State_StepTest::checkChange() {
    if (_lowState->userCmd == UserCommand::L2_B) {
        return FSMStateName::PASSIVE;
    }
    if (_lowState->userCmd == UserCommand::L2_A) {
        return FSMStateName::FIXEDSTAND;
    }
    if (_lowState->userCmd == UserCommand::L2_X || _isJumpCompleted) {
        return FSMStateName::TROTTING;
    }
    return FSMStateName::STEPTEST;
}

Vec3 State_StepTest::clampJointAngles(const Vec3& q) const {
    // StepTest 同样使用统一的机械限位。
    const Vec3 lower = _ctrlComp->parameters.joint_limits.lower();
    const Vec3 upper = _ctrlComp->parameters.joint_limits.upper();
    Vec3 clamped = q;
    for (int i = 0; i < 3; ++i) {
        clamped(i) = std::min(std::max(clamped(i), lower(i)), upper(i));
    }
    return clamped;
}
