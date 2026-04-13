#include "FSM/State_Trotting.h"
#include "FSM/StateMotorParams.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace {

template<typename T>
T clampValue(T value, T min_value, T max_value) {
    return std::min(std::max(value, min_value), max_value);
}

double applyDeadband(double value, double deadband) {
    if (std::fabs(value) <= deadband) {
        return 0.0;
    }
    const double normalized = (std::fabs(value) - deadband) / (1.0 - deadband);
    return std::copysign(normalized, value);
}

double mapAxisToSignedLimit(double axis_value, const Vec2& limits, double deadband) {
    const double normalized = applyDeadband(axis_value, deadband);
    if (normalized >= 0.0) {
        return normalized * limits(1);
    }
    return -std::fabs(normalized) * std::fabs(limits(0));
}

Vec3 rotateAroundBodyZ(const Vec3& point, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Vec3 rotated = point;
    rotated.x() = c * point.x() - s * point.y();
    rotated.y() = s * point.x() + c * point.y();
    return rotated;
}

double cycloidProgress(double phase) {
    const double theta = 2.0 * M_PI * clampValue(phase, 0.0, 1.0);
    return (theta - std::sin(theta)) / (2.0 * M_PI);
}

}  // namespace

State_Trotting::State_Trotting(CtrlComponents* ctrlComp)
    : FSMState(ctrlComp, FSMStateName::TROTTING, "trotting") {
    for (auto& foot : _enterFootPos) {
        foot.setZero();
    }
    for (auto& foot : _nominalFootPos) {
        foot.setZero();
    }
    for (auto& q : _lastLegQ) {
        q.setZero();
    }
}

double State_Trotting::getTimeSec() {
    using namespace std::chrono;
    static const steady_clock::time_point start = steady_clock::now();
    return duration<double>(steady_clock::now() - start).count();
}

void State_Trotting::enter() {
    std::cout << "[Trot] entering trotting state" << std::endl;

    for (int i = 0; i < 12; ++i) {
        _initMotorQ(i) = _lowState->motorState[i].q;
    }

    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 q = _initMotorQ.segment(leg * 3, 3);
        _lastLegQ[leg] = q;
        _enterFootPos[leg] = _ctrlComp->robotModel->forwardKinematics(q, leg, FrameType::HIP);
        _nominalFootPos[leg] = _ctrlComp->parameters.stand_targets.normal_feet_in_hip[leg];

        for (int j = 0; j < 3; ++j) {
            const int id = leg * 3 + j;
            fsm_motor_params::ApplyJointProfile(
                &_lowCmd->motorCmd[id], fsm_motor_params::kTrottingProfile[j]);
            _lowCmd->motorCmd[id].q = q(j);
        }
    }

    _motionParams = {};
    _accelLimitParams = {};
    _transitionCount = 0;
    _startTime = getTimeSec();
    _lastCommandUpdateTime = _startTime;
    _ctrlComp->setStartWave();
}

void State_Trotting::processJoystickInput() {
    const double lx = static_cast<double>(_lowState->userValue.lx);
    const double ly = static_cast<double>(_lowState->userValue.ly);
    const double rx = static_cast<double>(_lowState->userValue.rx);

    const double current_time = getTimeSec();
    const double dt = std::max(0.002, std::min(current_time - _lastCommandUpdateTime, 0.02));
    _lastCommandUpdateTime = current_time;

    const Vec2 limit_x = _ctrlComp->robotModel->getRobVelLimitX();
    const Vec2 limit_y = _ctrlComp->robotModel->getRobVelLimitY();
    const Vec2 limit_yaw = _ctrlComp->robotModel->getRobVelLimitYaw();

    const double velocity_x = mapAxisToSignedLimit(-ly, limit_x, 0.08);
    const double velocity_y = mapAxisToSignedLimit(lx, limit_y, 0.08);
    const double yaw_rate = mapAxisToSignedLimit(rx, limit_yaw, 0.08);

    _motionParams.joy = Vec2(ly, lx);
    applyAccelerationLimits(velocity_x, velocity_y, yaw_rate, dt);
}

void State_Trotting::applyAccelerationLimits(double velocity_x, double velocity_y, double yaw_rate, double dt) {
    const double max_delta_x = _limitParams.maxAccelX * dt;
    const double max_delta_y = _limitParams.maxAccelY * dt;
    const double max_delta_yaw = _limitParams.maxAccelYaw * dt;

    const double delta_x =
        clampValue(velocity_x - _accelLimitParams.lastVelocityX, -max_delta_x, max_delta_x);
    const double delta_y =
        clampValue(velocity_y - _accelLimitParams.lastVelocityY, -max_delta_y, max_delta_y);
    const double delta_yaw =
        clampValue(yaw_rate - _accelLimitParams.lastYawRate, -max_delta_yaw, max_delta_yaw);

    _motionParams.velocityX = _accelLimitParams.lastVelocityX + delta_x;
    _motionParams.velocityY = _accelLimitParams.lastVelocityY + delta_y;
    _motionParams.yawRate = _accelLimitParams.lastYawRate + delta_yaw;

    _accelLimitParams.lastVelocityX = _motionParams.velocityX;
    _accelLimitParams.lastVelocityY = _motionParams.velocityY;
    _accelLimitParams.lastYawRate = _motionParams.yawRate;
}

bool State_Trotting::hasActiveMotionCommand() const {
    return std::fabs(_motionParams.velocityX) > MOTION_EPS ||
           std::fabs(_motionParams.velocityY) > MOTION_EPS ||
           std::fabs(_motionParams.yawRate) > MOTION_EPS;
}

Vec3 State_Trotting::computeFrontFoothold(int leg) const {
    const double stance_time = CYCLE_T * 0.5;

    Vec3 translation_step = Vec3::Zero();
    translation_step.x() = _motionParams.velocityX * stance_time;
    translation_step.y() = _motionParams.velocityY * stance_time;

    const Vec3 nominal_body = _ctrlComp->parameters.hip_mounts_in_body[leg] + _nominalFootPos[leg];
    const double half_yaw = _motionParams.yawRate * stance_time * 0.5;
    const Vec3 yaw_offset = rotateAroundBodyZ(nominal_body, half_yaw) - nominal_body;

    return _nominalFootPos[leg] + 0.5 * translation_step + yaw_offset;
}

Vec3 State_Trotting::computeRearFoothold(int leg) const {
    const double stance_time = CYCLE_T * 0.5;

    Vec3 translation_step = Vec3::Zero();
    translation_step.x() = _motionParams.velocityX * stance_time;
    translation_step.y() = _motionParams.velocityY * stance_time;

    const Vec3 nominal_body = _ctrlComp->parameters.hip_mounts_in_body[leg] + _nominalFootPos[leg];
    const double half_yaw = _motionParams.yawRate * stance_time * 0.5;
    const Vec3 yaw_offset = rotateAroundBodyZ(nominal_body, -half_yaw) - nominal_body;

    return _nominalFootPos[leg] - 0.5 * translation_step + yaw_offset;
}

Vec3 State_Trotting::computeSwingFootTarget(int leg, double phase) const {
    const Vec3 rear = computeRearFoothold(leg);
    const Vec3 front = computeFrontFoothold(leg);
    const double alpha = cycloidProgress(phase);
    const double theta = 2.0 * M_PI * clampValue(phase, 0.0, 1.0);

    Vec3 target = rear + alpha * (front - rear);
    target.z() += LIFT_H * (1.0 - std::cos(theta)) * 0.5;
    return target;
}

Vec3 State_Trotting::computeStanceFootTarget(int leg, double phase) const {
    const Vec3 front = computeFrontFoothold(leg);
    const Vec3 rear = computeRearFoothold(leg);
    return front + clampValue(phase, 0.0, 1.0) * (rear - front);
}

void State_Trotting::generateLegTrajectory(int leg,
                                           double masterT,
                                           double trans,
                                           Vec12& cmd,
                                           VecInt4& contact,
                                           Vec4& phase) {
    // 对角腿相差半周期，对应 trot 的经典 FR/RL 与 FL/RR 交替。
    const double legT = std::fmod(masterT + LEG_PHASE[leg] * CYCLE_T, CYCLE_T);
    const double normalized = legT / CYCLE_T;
    const bool swing = (normalized < 0.5);
    const double phase_in_segment = swing ? normalized / 0.5 : (normalized - 0.5) / 0.5;

    contact(leg) = swing ? 0 : 1;
    phase(leg) = phase_in_segment;

    const Vec3 gait_target = swing
        ? computeSwingFootTarget(leg, phase_in_segment)
        : computeStanceFootTarget(leg, phase_in_segment);
    const Vec3 foot_target = (1.0 - trans) * _enterFootPos[leg] + trans * gait_target;
    calculateIKAndApply(leg, foot_target, cmd);
}

void State_Trotting::calculateIKAndApply(int leg, const Vec3& target_foot_in_hip, Vec12& cmd) {
    // 足端目标先转关节角，再限制单拍最大跳变，避免指令突变。
    Vec3 q_des = _ctrlComp->robotModel->inverseKinematics(target_foot_in_hip, leg, FrameType::HIP);
    q_des = clampJointAngles(q_des);

    Vec3 delta = q_des - _lastLegQ[leg];
    const double max_delta = 0.03;
    if (delta.norm() > max_delta) {
        delta = max_delta * delta.normalized();
    }

    const Vec3 q_cmd = _lastLegQ[leg] + delta;
    _lastLegQ[leg] = q_cmd;
    cmd.segment(leg * 3, 3) = q_cmd;
}

Vec3 State_Trotting::clampJointAngles(const Vec3& angles) const {
    // trotting 中所有 IK 结果都会经过限位裁剪。
    const Vec3 lower = _ctrlComp->parameters.joint_limits.lower();
    const Vec3 upper = _ctrlComp->parameters.joint_limits.upper();
    Vec3 clamped = angles;
    for (int i = 0; i < 3; ++i) {
        clamped(i) = clampValue(clamped(i), lower(i), upper(i));
    }
    return clamped;
}

void State_Trotting::run() {
    if (_transitionCount < 100) {
        ++_transitionCount;
    }

    processJoystickInput();

    const double trans = std::min(1.0, static_cast<double>(_transitionCount) / 100.0);
    Vec12 cmd = Vec12::Zero();
    VecInt4 contact = VecInt4::Ones();
    Vec4 phase = Vec4::Constant(0.5);

    if (!hasActiveMotionCommand()) {
        for (int leg = 0; leg < 4; ++leg) {
            const Vec3 foot_target = (1.0 - trans) * _enterFootPos[leg] + trans * _nominalFootPos[leg];
            calculateIKAndApply(leg, foot_target, cmd);
        }
        _ctrlComp->setContactPhase(contact, phase);
        _lowCmd->setQ(cmd);
        return;
    }

    const double masterT = std::fmod(getTimeSec() - _startTime, CYCLE_T);
    contact = VecInt4::Zero();

    // 当前 contact / phase 由状态内部维护，再写回上下文给估计器使用。
    for (int leg = 0; leg < 4; ++leg) {
        generateLegTrajectory(leg, masterT, trans, cmd, contact, phase);
    }

    _ctrlComp->setContactPhase(contact, phase);
    _lowCmd->setQ(cmd);
}

void State_Trotting::exit() {
    for (int i = 0; i < 50; ++i) {
        Vec12 tmp = _initMotorQ;
        for (int leg = 0; leg < 4; ++leg) {
            const int id = leg * 3;
            tmp(id) = HIP_JOINT_FIXED;
            tmp(id + 1) = tmp(id + 1) * 0.9 + _initMotorQ(id + 1) * 0.1;
            tmp(id + 2) = tmp(id + 2) * 0.9 + _initMotorQ(id + 2) * 0.1;
        }
        _lowCmd->setQ(tmp);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    _lowCmd->setQ(_initMotorQ);
}

FSMStateName State_Trotting::checkChange() {
    if (_lowState->userCmd == UserCommand::L2_B) {
        return FSMStateName::PASSIVE;
    }
    if (_lowState->userCmd == UserCommand::L2_A) {
        return FSMStateName::FIXEDSTAND;
    }
    if (_lowState->userCmd == UserCommand::L2_Y) {
        return FSMStateName::STEPTEST;
    }
    return FSMStateName::TROTTING;
}
