#include "FSM/State_Trotting.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace {

template<typename T>
T clampValue(T value, T min_value, T max_value) {
    return std::min(std::max(value, min_value), max_value);
}

}  // namespace

State_Trotting::State_Trotting(CtrlComponents* ctrlComp)
    : FSMState(ctrlComp, FSMStateName::TROTTING, "trotting") {
    for (auto& foot : _initFootPos) {
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

Vec3 State_Trotting::cycloidTraj3D(double phase) const {
    const double theta = phase * 4.0 * M_PI;
    const bool swing = (theta <= 2.0 * M_PI);
    const double progress = swing
        ? (theta - std::sin(theta)) / (2.0 * M_PI)
        : (1.0 - (theta - 2.0 * M_PI) / (2.0 * M_PI));

    Vec3 trajectory = Vec3::Zero();
    trajectory.x() = _motionParams.stepLenX * (progress - 0.5);
    trajectory.y() = _motionParams.stepLenY * (progress - 0.5);
    if (swing && (std::fabs(_motionParams.stepLenX) > 1e-6 || std::fabs(_motionParams.stepLenY) > 1e-6)) {
        trajectory.z() = LIFT_H * (1.0 - std::cos(theta)) / 2.0;
    }
    return trajectory;
}

Vec3 State_Trotting::yawCycloidTraj3D(int leg, double phase, bool swing) const {
    if (!swing) {
        return Vec3::Zero();
    }

    const double theta = phase * 4.0 * M_PI;
    const double yaw_total = _motionParams.omega * CYCLE_T * 0.5;
    const double progress = (theta - std::sin(theta)) / (2.0 * M_PI);
    const double yaw_progress = yaw_total * progress;

    const Vec3& initial = _initFootPos[leg];
    const double c = std::cos(yaw_progress);
    const double s = std::sin(yaw_progress);

    Vec3 rotated;
    rotated.x() = c * initial.x() - s * initial.y();
    rotated.y() = s * initial.x() + c * initial.y();
    rotated.z() = initial.z() + LIFT_H * (1.0 - std::cos(theta)) / 2.0;
    return rotated - initial;
}

void State_Trotting::enter() {
    std::cout << "[Trot] entering trotting state" << std::endl;

    for (int i = 0; i < 12; ++i) {
        _initMotorQ(i) = _lowState->motorState[i].q;
    }

    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 q = _initMotorQ.segment(leg * 3, 3);
        // 进入状态时先记录当前足端位置，后续所有轨迹都相对这个锚点展开。
        _lastLegQ[leg] = q;
        _initFootPos[leg] = _ctrlComp->robotModel->forwardKinematics(q, leg, FrameType::HIP);
        for (int j = 0; j < 3; ++j) {
            const int id = leg * 3 + j;
            _lowCmd->motorCmd[id].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            _lowCmd->motorCmd[id].dq = 0.0f;
            _lowCmd->motorCmd[id].tau = 0.05f;
            _lowCmd->motorCmd[id].Kp = 6.2f;
            _lowCmd->motorCmd[id].Kd = 0.2f;
            _lowCmd->motorCmd[id].q = q(j);
        }
    }

    _motionParams = {};
    _accelLimitParams = {};
    _transitionCount = 0;
    _startTime = getTimeSec();
    _ctrlComp->setStartWave();
}

void State_Trotting::processJoystickInput() {
    const double lx = static_cast<double>(_lowState->userValue.lx);
    const double ly = static_cast<double>(_lowState->userValue.ly);
    const double rx = static_cast<double>(_lowState->userValue.rx);

    static double lastTime = getTimeSec();
    const double current_time = getTimeSec();
    const double dt = std::max(0.002, std::min(current_time - lastTime, 0.01));
    lastTime = current_time;

    const double rx_dead = 0.1;
    _motionParams.omega = (std::fabs(rx) > rx_dead)
        ? std::copysign(std::fabs(rx) - rx_dead, rx) / (1.0 - rx_dead) * MAX_OMEGA
        : 0.0;

    // 摇杆死区和最大步长保持旧策略风格，只是输入来源换成统一的 lowState。
    const double dead_x = 0.1;
    const double dead_y = 0.2;
    double raw_step_x = (std::fabs(ly) > dead_x)
        ? std::copysign((std::fabs(ly) - dead_x) / (1.0 - dead_x), -ly) * MAX_SWING_X
        : 0.0;
    double raw_step_y = (std::fabs(lx) > dead_y)
        ? std::copysign((std::fabs(lx) - dead_y) / (1.0 - dead_y), lx) * MAX_SWING_Y
        : 0.0;

    _motionParams.joy = Vec2(ly, lx);
    applyAccelerationLimits(raw_step_x, raw_step_y, dt);
}

void State_Trotting::applyAccelerationLimits(double& stepLenX, double& stepLenY, double dt) {
    const double max_delta_x = _limitParams.maxAccelX * dt;
    const double max_delta_y = _limitParams.maxAccelY * dt;
    const double delta_x = clampValue(stepLenX - _accelLimitParams.lastStepLenX, -max_delta_x, max_delta_x);
    const double delta_y = clampValue(stepLenY - _accelLimitParams.lastStepLenY, -max_delta_y, max_delta_y);
    _motionParams.stepLenX = _accelLimitParams.lastStepLenX + delta_x;
    _motionParams.stepLenY = _accelLimitParams.lastStepLenY + delta_y;
    _accelLimitParams.lastStepLenX = _motionParams.stepLenX;
    _accelLimitParams.lastStepLenY = _motionParams.stepLenY;
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

    Vec3 relative_target = Vec3::Zero();
    const bool has_xy = std::fabs(_motionParams.stepLenX) > 1e-6 || std::fabs(_motionParams.stepLenY) > 1e-6;
    const bool has_yaw = std::fabs(_motionParams.omega) > 1e-3;
    if (has_xy) {
        relative_target += cycloidTraj3D(normalized);
    }
    if (has_yaw && !has_xy) {
        relative_target += yawCycloidTraj3D(leg, normalized, swing);
    }

    const Vec3 foot_target = _initFootPos[leg] + relative_target * trans;
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

    const double masterT = std::fmod(getTimeSec() - _startTime, CYCLE_T);
    const double trans = std::min(1.0, static_cast<double>(_transitionCount) / 100.0);
    Vec12 cmd = _initMotorQ;
    VecInt4 contact = VecInt4::Zero();
    Vec4 phase = Vec4::Constant(0.5);

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
