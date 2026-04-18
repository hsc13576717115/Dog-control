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

double wrapAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double applyDeadband(double value, double deadband) {
    if (std::fabs(value) <= deadband) {
        return 0.0;
    }
    const double normalized = (std::fabs(value) - deadband) / (1.0 - deadband);
    return std::copysign(normalized, value);
}

double applySignedExpo(double normalized_value, double expo) {
    if (std::fabs(normalized_value) < 1e-9) {
        return 0.0;
    }
    const double safe_expo = std::max(1.0, expo);
    return std::copysign(std::pow(std::fabs(normalized_value), safe_expo), normalized_value);
}

double mapAxisToSignedLimit(double axis_value, const Vec2& limits, double deadband, double expo) {
    const double normalized = applyDeadband(axis_value, deadband);
    const double shaped = applySignedExpo(normalized, expo);
    if (shaped >= 0.0) {
        return shaped * limits(1);
    }
    return std::copysign(std::fabs(shaped) * std::fabs(limits(0)), shaped);
}

double cycloidBlend(double phase) {
    const double s = clampValue(phase, 0.0, 1.0);
    return (2.0 * M_PI * s - std::sin(2.0 * M_PI * s)) / (2.0 * M_PI);
}

double cycloidLiftOffset(double phase, double lift_height) {
    const double s = clampValue(phase, 0.0, 1.0);
    return 0.5 * lift_height * (1.0 - std::cos(2.0 * M_PI * s));
}

Vec3 clampPlanarShift(const Vec3& shift, double max_x, double max_y) {
    Vec3 clamped = shift;
    clamped.x() = clampValue(clamped.x(), -max_x, max_x);
    clamped.y() = clampValue(clamped.y(), -max_y, max_y);
    clamped.z() = 0.0;
    return clamped;
}

}  // namespace

State_Trotting::State_Trotting(CtrlComponents* ctrlComp)
    : FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"),
      _joyMapping(ctrlComp->parameters.joy_mapping),
      _trotParams(ctrlComp->parameters.trot) {
    for (auto& foot : _enterFootPos) {
        foot.setZero();
    }
    for (auto& foot : _nominalFootPos) {
        foot.setZero();
    }
    for (auto& q : _lastLegQ) {
        q.setZero();
    }
    for (auto& swing_state : _prevSwingState) {
        swing_state = false;
    }
    for (auto& foot : _swingStartFootPos) {
        foot.setZero();
    }
    for (auto& foot : _swingTargetFootPos) {
        foot.setZero();
    }
    for (auto& foot : _stanceStartFootPos) {
        foot.setZero();
    }
}

double State_Trotting::getTimeSec() {
    using namespace std::chrono;
    static const steady_clock::time_point start = steady_clock::now();
    return duration<double>(steady_clock::now() - start).count();
}

Vec3 State_Trotting::footHipToBodyFrame(int leg, const Vec3& foot_in_hip) const {
    return _ctrlComp->parameters.hip_mounts_in_body[leg] + foot_in_hip;
}

Vec3 State_Trotting::footBodyToHipFrame(int leg, const Vec3& foot_in_body) const {
    return foot_in_body - _ctrlComp->parameters.hip_mounts_in_body[leg];
}

Vec3 State_Trotting::rotateBodyPointYaw(const Vec3& point_body, double yaw_delta) const {
    const double c = std::cos(yaw_delta);
    const double s = std::sin(yaw_delta);

    Vec3 rotated = point_body;
    rotated.x() = c * point_body.x() - s * point_body.y();
    rotated.y() = s * point_body.x() + c * point_body.y();
    return rotated;
}

State_Trotting::LegPhaseState State_Trotting::computeLegPhaseState(int leg, double masterT) const {
    const double cycle_time = std::max(0.1, _trotParams.cycle_time);
    const double stance_ratio = clampValue(_trotParams.stance_ratio, 0.15, 0.85);
    const double swing_ratio = 1.0 - stance_ratio;

    double cycle_phase = std::fmod(masterT / cycle_time + LEG_PHASE[leg], 1.0);
    if (cycle_phase < 0.0) {
        cycle_phase += 1.0;
    }

    LegPhaseState state;
    state.cyclePhase = cycle_phase;
    state.stanceTime = cycle_time * stance_ratio;
    state.swingTime = cycle_time * swing_ratio;
    state.swing = cycle_phase >= stance_ratio;
    if (state.swing) {
        state.segmentPhase = (cycle_phase - stance_ratio) / swing_ratio;
        state.remainingSwingTime = (1.0 - state.segmentPhase) * state.swingTime;
    } else {
        state.segmentPhase = cycle_phase / stance_ratio;
        state.remainingSwingTime = 0.0;
    }
    return state;
}

Vec3 State_Trotting::nominalFootBodyPosition(int leg) const {
    return footHipToBodyFrame(leg, _nominalFootPos[leg]);
}

Vec3 State_Trotting::projectBodyPointToRotationCircle(int leg, const Vec3& point_body) const {
    const Vec3 nominal_body = nominalFootBodyPosition(leg);
    const double radius = std::hypot(nominal_body.x(), nominal_body.y());
    double theta = std::atan2(point_body.y(), point_body.x());

    if (std::hypot(point_body.x(), point_body.y()) < 1e-6) {
        theta = std::atan2(nominal_body.y(), nominal_body.x());
    }

    Vec3 projected = point_body;
    projected.x() = radius * std::cos(theta);
    projected.y() = radius * std::sin(theta);
    return projected;
}

Vec3 State_Trotting::computeSymmetricHalfStepShift(double stance_time) const {
    return Vec3(_motionParams.velocityX, _motionParams.velocityY, 0.0) * (0.5 * stance_time);
}

Vec3 State_Trotting::computeTouchdownFootBodyTarget(int leg, double stance_time) const {
    const Vec3 nominal_body = nominalFootBodyPosition(leg);
    const Vec3 linear_half_shift = computeSymmetricHalfStepShift(stance_time);
    const Vec3 rotated_nominal = rotateBodyPointYaw(nominal_body, 0.5 * _motionParams.yawRate * stance_time);
    Vec3 target_body = rotated_nominal + linear_half_shift;
    target_body.z() = nominal_body.z();
    return target_body;
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
        _swingStartFootPos[leg] = _enterFootPos[leg];
        _swingTargetFootPos[leg] = _nominalFootPos[leg];
        _stanceStartFootPos[leg] = _enterFootPos[leg];
        _prevSwingState[leg] = false;

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

    const Vec2 robot_limit_x = _ctrlComp->robotModel->getRobVelLimitX();
    const Vec2 robot_limit_y = _ctrlComp->robotModel->getRobVelLimitY();
    const Vec2 robot_limit_yaw = _ctrlComp->robotModel->getRobVelLimitYaw();

    const Vec2 joy_limit_x(-std::fabs(_joyMapping.max_vx), std::fabs(_joyMapping.max_vx));
    const Vec2 joy_limit_y(-std::fabs(_joyMapping.max_vy), std::fabs(_joyMapping.max_vy));
    const Vec2 joy_limit_yaw(-std::fabs(_joyMapping.max_yaw), std::fabs(_joyMapping.max_yaw));

    const Vec2 effective_limit_x(
        std::max(robot_limit_x(0), joy_limit_x(0)),
        std::min(robot_limit_x(1), joy_limit_x(1)));
    const Vec2 effective_limit_y(
        std::max(robot_limit_y(0), joy_limit_y(0)),
        std::min(robot_limit_y(1), joy_limit_y(1)));
    const Vec2 effective_limit_yaw(
        std::max(robot_limit_yaw(0), joy_limit_yaw(0)),
        std::min(robot_limit_yaw(1), joy_limit_yaw(1)));

    const double velocity_x =
        mapAxisToSignedLimit(-ly, effective_limit_x, _joyMapping.deadband, _joyMapping.expo);
    const double velocity_y =
        mapAxisToSignedLimit(lx, effective_limit_y, _joyMapping.deadband, _joyMapping.expo);
    const double yaw_rate =
        mapAxisToSignedLimit(rx, effective_limit_yaw, _joyMapping.deadband, _joyMapping.expo);

    _motionParams.joy << ly, lx;
    applyAccelerationLimits(velocity_x, velocity_y, yaw_rate, dt);

    _motionParams.velocityX = clampValue(_motionParams.velocityX, effective_limit_x(0), effective_limit_x(1));
    _motionParams.velocityY = clampValue(_motionParams.velocityY, effective_limit_y(0), effective_limit_y(1));
    _motionParams.yawRate = clampValue(_motionParams.yawRate, effective_limit_yaw(0), effective_limit_yaw(1));
}

void State_Trotting::applyAccelerationLimits(double velocity_x, double velocity_y, double yaw_rate, double dt) {
    const double max_delta_x = _joyMapping.max_accel_x * dt;
    const double max_delta_y = _joyMapping.max_accel_y * dt;
    const double max_delta_yaw = _joyMapping.max_accel_yaw * dt;

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

bool State_Trotting::isPureRotationCommand() const {
    const double xy_motion = std::hypot(_motionParams.velocityX, _motionParams.velocityY);
    return xy_motion < _trotParams.pure_rotation_translation_eps &&
           std::fabs(_motionParams.yawRate) > _trotParams.pure_yaw_threshold;
}

Vec3 State_Trotting::clampFootholdToWorkspace(int leg, const Vec3& foothold_in_hip) const {
    const Vec3 shift = foothold_in_hip - _nominalFootPos[leg];
    const Vec3 clamped_shift = clampPlanarShift(
        shift, _trotParams.max_foothold_shift_x, _trotParams.max_foothold_shift_y);

    Vec3 clamped = _nominalFootPos[leg] + clamped_shift;
    clamped.z() = _nominalFootPos[leg].z();
    return clamped;
}

Vec3 State_Trotting::computePureRotationSwingFootTarget(int leg, double phase) const {
    const double xy_alpha = cycloidBlend(phase);
    const Vec3 start_body =
        projectBodyPointToRotationCircle(leg, footHipToBodyFrame(leg, _swingStartFootPos[leg]));
    const Vec3 end_body =
        projectBodyPointToRotationCircle(leg, footHipToBodyFrame(leg, _swingTargetFootPos[leg]));

    const double start_theta = std::atan2(start_body.y(), start_body.x());
    const double end_theta = std::atan2(end_body.y(), end_body.x());
    const double delta_theta = wrapAngle(end_theta - start_theta);
    const double radius = std::hypot(nominalFootBodyPosition(leg).x(), nominalFootBodyPosition(leg).y());
    const double theta = start_theta + xy_alpha * delta_theta;

    Vec3 target_body = start_body;
    target_body.x() = radius * std::cos(theta);
    target_body.y() = radius * std::sin(theta);

    Vec3 target = footBodyToHipFrame(leg, target_body);
    target.z() = _swingStartFootPos[leg].z() +
                 (_swingTargetFootPos[leg].z() - _swingStartFootPos[leg].z()) * xy_alpha +
                 cycloidLiftOffset(phase, _trotParams.lift_height);
    return target;
}

Vec3 State_Trotting::computeSwingFootTarget(int leg, double phase) const {
    if (isPureRotationCommand()) {
        return computePureRotationSwingFootTarget(leg, phase);
    }

    const double xy_alpha = cycloidBlend(phase);
    const Vec3 start_body = footHipToBodyFrame(leg, _swingStartFootPos[leg]);
    const Vec3 end_body = footHipToBodyFrame(leg, _swingTargetFootPos[leg]);
    const Vec3 target_body = start_body + xy_alpha * (end_body - start_body);

    Vec3 target = footBodyToHipFrame(leg, target_body);
    target.z() = _swingStartFootPos[leg].z() +
                 (_swingTargetFootPos[leg].z() - _swingStartFootPos[leg].z()) * xy_alpha +
                 cycloidLiftOffset(phase, _trotParams.lift_height);
    return target;
}

Vec3 State_Trotting::computeStanceFootTarget(int leg, const LegPhaseState& phase_state) const {
    const double s = clampValue(phase_state.segmentPhase, 0.0, 1.0);
    const Vec3 nominal_body = nominalFootBodyPosition(leg);
    const double centered_phase = 0.5 - s;
    Vec3 target_body = rotateBodyPointYaw(
        nominal_body, centered_phase * _motionParams.yawRate * phase_state.stanceTime);
    target_body += centered_phase * phase_state.stanceTime *
                   Vec3(_motionParams.velocityX, _motionParams.velocityY, 0.0);
    Vec3 target = footBodyToHipFrame(leg, target_body);
    target.z() = _nominalFootPos[leg].z();
    return target;
}

void State_Trotting::updateLegPhaseAnchors(int leg, const LegPhaseState& phase_state) {
    if (phase_state.swing == _prevSwingState[leg]) {
        return;
    }

    if (phase_state.swing) {
        const Vec3 q = _lowState->getQ().col(leg);
        _swingStartFootPos[leg] =
            _ctrlComp->robotModel->forwardKinematics(q, leg, FrameType::HIP);
        const Vec3 landing_body = computeTouchdownFootBodyTarget(leg, phase_state.stanceTime);
        _swingTargetFootPos[leg] = clampFootholdToWorkspace(leg, footBodyToHipFrame(leg, landing_body));
    } else {
        _stanceStartFootPos[leg] = _swingTargetFootPos[leg];
    }

    _prevSwingState[leg] = phase_state.swing;
}

void State_Trotting::syncAnchorsForStanding(const std::array<Vec3, 4>& foot_targets) {
    for (int leg = 0; leg < 4; ++leg) {
        _prevSwingState[leg] = false;
        _swingStartFootPos[leg] = foot_targets[leg];
        _swingTargetFootPos[leg] = foot_targets[leg];
        _stanceStartFootPos[leg] = foot_targets[leg];
    }
}

void State_Trotting::generateLegTrajectory(int leg,
                                           double masterT,
                                           double trans,
                                           Vec12& cmd,
                                           VecInt4& contact,
                                           Vec4& phase) {
    const LegPhaseState phase_state = computeLegPhaseState(leg, masterT);
    updateLegPhaseAnchors(leg, phase_state);

    contact(leg) = phase_state.swing ? 0 : 1;
    phase(leg) = phase_state.segmentPhase;

    const Vec3 gait_target = phase_state.swing
        ? computeSwingFootTarget(leg, phase_state.segmentPhase)
        : computeStanceFootTarget(leg, phase_state);
    const Vec3 foot_target = (1.0 - trans) * _enterFootPos[leg] + trans * gait_target;
    calculateIKAndApply(leg, foot_target, cmd);
}

void State_Trotting::calculateIKAndApply(int leg, const Vec3& target_foot_in_hip, Vec12& cmd) {
    Vec3 q_des = _ctrlComp->robotModel->inverseKinematics(target_foot_in_hip, leg, FrameType::HIP);
    q_des = clampJointAngles(q_des);

    Vec3 delta = q_des - _lastLegQ[leg];
    const double max_delta = std::max(1e-4, _trotParams.max_joint_delta);
    if (delta.norm() > max_delta) {
        delta = max_delta * delta.normalized();
    }

    const Vec3 q_cmd = _lastLegQ[leg] + delta;
    _lastLegQ[leg] = q_cmd;
    cmd.segment(leg * 3, 3) = q_cmd;
}

Vec3 State_Trotting::clampJointAngles(const Vec3& angles) const {
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
        std::array<Vec3, 4> standing_targets{};
        for (int leg = 0; leg < 4; ++leg) {
            const Vec3 foot_target = (1.0 - trans) * _enterFootPos[leg] + trans * _nominalFootPos[leg];
            standing_targets[leg] = foot_target;
            calculateIKAndApply(leg, foot_target, cmd);
        }
        syncAnchorsForStanding(standing_targets);
        _ctrlComp->setContactPhase(contact, phase);
        _lowCmd->setQ(cmd);
        return;
    }

    const double masterT = getTimeSec() - _startTime;
    contact = VecInt4::Zero();

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
