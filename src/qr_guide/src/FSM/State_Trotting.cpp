/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include "FSM/StateMotorParams.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "control/BalanceCtrl.h"
#include "control/Estimator.h"
#include "common/mathTools.h"
#include "common/unitreeRobot.h"

namespace {

template<typename T>
T clampValue(T value, T min_value, T max_value) {
    return std::min(std::max(value, min_value), max_value);
}

double wrapAngle(double angle) {
    while (angle > M_PI) { angle -= 2.0 * M_PI; }
    while (angle < -M_PI) { angle += 2.0 * M_PI; }
    return angle;
}

double applyDeadband(double value, double deadband) {
    if (std::fabs(value) <= deadband) { return 0.0; }
    const double normalized = (std::fabs(value) - deadband) / (1.0 - deadband);
    return std::copysign(normalized, value);
}

double applySignedExpo(double normalized_value, double expo) {
    if (std::fabs(normalized_value) < 1e-9) { return 0.0; }
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

double cycloidBlendVelocityScale(double phase, double swing_time) {
    const double s = clampValue(phase, 0.0, 1.0);
    return (1.0 - std::cos(2.0 * M_PI * s)) / std::max(1e-4, swing_time);
}

double asymmetricCycloidLiftOffset(double phase, double lift_height, double peak_phase) {
    const double s = clampValue(phase, 0.0, 1.0);
    const double peak = clampValue(peak_phase, 0.15, 0.85);
    if (s <= peak) {
        const double u = s / peak;
        return 0.5 * lift_height * (1.0 - std::cos(M_PI * u));
    }
    const double u = (s - peak) / (1.0 - peak);
    return 0.5 * lift_height * (1.0 + std::cos(M_PI * u));
}

double asymmetricCycloidLiftVelocity(double phase,
                                     double lift_height,
                                     double swing_time,
                                     double peak_phase) {
    const double s = clampValue(phase, 0.0, 1.0);
    const double peak = clampValue(peak_phase, 0.15, 0.85);
    const double safe_swing_time = std::max(1e-4, swing_time);
    if (s <= peak) {
        const double u = s / peak;
        return 0.5 * lift_height * M_PI * std::sin(M_PI * u) / (peak * safe_swing_time);
    }
    const double u = (s - peak) / (1.0 - peak);
    return -0.5 * lift_height * M_PI * std::sin(M_PI * u) /
           ((1.0 - peak) * safe_swing_time);
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
      _trotParams(ctrlComp->parameters.trot),
      _est(ctrlComp->estimator.get()),
      _balCtrl(ctrlComp->balCtrl.get()),
      _robModel(ctrlComp->robotModel.get()) {
    for (auto& foot : _enterFootPos) { foot.setZero(); }
    for (auto& foot : _nominalFootPos) { foot.setZero(); }
    for (auto& q : _lastLegQ) { q.setZero(); }
    for (auto& swing_state : _prevSwingState) { swing_state = false; }
    for (auto& foot : _swingStartFootPos) { foot.setZero(); }
    for (auto& foot : _swingTargetFootPos) { foot.setZero(); }
    for (auto& foot : _stanceStartFootPos) { foot.setZero(); }
    for (auto& foot : _idleBlendStartFootPos) { foot.setZero(); }

    const auto& f = ctrlComp->parameters.force;
    // Body position tracking (x, y, z)
    _Kpp = f.kp_body_xyz.asDiagonal();
    _Kdp = f.kd_body_xyz.asDiagonal();
    // Body orientation tracking (roll, pitch, yaw)
    _Kpw = f.kp_body_rpy.asDiagonal();
    _Kdw = f.kd_body_rpy.asDiagonal();
    // Swing leg tracking
    _KpSwing = f.kp_swing.asDiagonal();
    _KdSwing = f.kd_swing.asDiagonal();
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
    const double stance_ratio = 0.5;
    const double swing_ratio = 0.5;

    double cycle_phase = std::fmod(masterT / cycle_time + LEG_PHASE[leg], 1.0);
    if (cycle_phase < 0.0) { cycle_phase += 1.0; }

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

Vec3 State_Trotting::computeTouchdownFootBodyTarget(int leg, const LegPhaseState& phase_state) const {
    const Vec3 nominal_body = nominalFootBodyPosition(leg);

    // The touchdown point is exactly the next stance start. With stance phase
    // s in [0, 1], s=0.5 is the nominal standing foot pose.
    const double stance_start_centered_phase = 0.5;
    const double trajectory_gain = std::max(0.0, _trotParams.stance_trajectory_gain);
    const double yaw_delta = trajectory_gain * stance_start_centered_phase *
                             _motionParams.yawRate * phase_state.stanceTime;
    Vec3 target_body = rotateBodyPointYaw(nominal_body, yaw_delta);
    target_body += trajectory_gain * stance_start_centered_phase * phase_state.stanceTime *
                   Vec3(_motionParams.velocityX, _motionParams.velocityY, 0.0);
    target_body.z() = nominal_body.z();
    return target_body;
}

void State_Trotting::enter() {
    std::cout << "[Trot] entering trotting state force_mode=" << _trotParams.force_mode
              << " default=idle_hold" << std::endl;

    for (int i = 0; i < 12; ++i) {
        _initMotorQ(i) = _lowState->motorState[i].q;
    }

    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 q = _initMotorQ.segment(leg * 3, 3);
        _lastLegQ[leg] = q;
        _enterFootPos[leg] = _robModel->forwardKinematics(q, leg, FrameType::HIP);
        _nominalFootPos[leg] = _ctrlComp->parameters.stand_targets.normal_feet_in_hip[leg];
        _swingStartFootPos[leg] = _enterFootPos[leg];
        _swingTargetFootPos[leg] = _nominalFootPos[leg];
        _stanceStartFootPos[leg] = _enterFootPos[leg];
        _idleBlendStartFootPos[leg] = _enterFootPos[leg];
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
    _gaitActive = false;
    _torqueSafetyInitialized = false;
    _forceRateInitialized = false;
    _transitionCount = 0;
    _startTime = getTimeSec();
    _commandActiveSince = -1.0;
    _gaitStartTime = _startTime;
    _lastDebugPrintTime = _startTime;
    _lastCommandUpdateTime = _startTime;
    _idleBlendStartTime = _startTime;

    // Initialize force control state
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();
    _tauPrev.setZero();
    _tau.setZero();
    _forceFeetBody.setZero();
    _forceFeetGlobal.setZero();
    _lastForceFeetBody.setZero();

    _ctrlComp->setAllStance();
}

void State_Trotting::processJoystickInput() {
    const double lx = static_cast<double>(_lowState->userValue.lx);
    const double ly = static_cast<double>(_lowState->userValue.ly);
    const double rx = static_cast<double>(_lowState->userValue.rx);

    const double current_time = getTimeSec();
    const double dt = std::max(0.002, std::min(current_time - _lastCommandUpdateTime, 0.02));
    _lastCommandUpdateTime = current_time;

    const Vec2 robot_limit_x = _robModel->getRobVelLimitX();
    const Vec2 robot_limit_y = _robModel->getRobVelLimitY();
    const Vec2 robot_limit_yaw = _robModel->getRobVelLimitYaw();

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
        mapAxisToSignedLimit(rx, effective_limit_y, _joyMapping.deadband, _joyMapping.expo);
    const double yaw_rate =
        mapAxisToSignedLimit(lx, effective_limit_yaw, _joyMapping.deadband, _joyMapping.expo);

    _motionParams.joy << ly, rx;
    applyAccelerationLimits(velocity_x, velocity_y, yaw_rate, dt);

    _motionParams.velocityX = clampValue(_motionParams.velocityX, effective_limit_x(0), effective_limit_x(1));
    _motionParams.velocityY = clampValue(_motionParams.velocityY, effective_limit_y(0), effective_limit_y(1));
    _motionParams.yawRate = clampValue(_motionParams.yawRate, effective_limit_yaw(0), effective_limit_yaw(1));
}

void State_Trotting::applyAccelerationLimits(double velocity_x, double velocity_y, double yaw_rate, double dt) {
    const double max_delta_x = _joyMapping.max_accel_x * dt;
    const double max_delta_y = _joyMapping.max_accel_y * dt;
    const double max_delta_yaw = _joyMapping.max_accel_yaw * dt;

    const double delta_x = clampValue(velocity_x - _accelLimitParams.lastVelocityX, -max_delta_x, max_delta_x);
    const double delta_y = clampValue(velocity_y - _accelLimitParams.lastVelocityY, -max_delta_y, max_delta_y);
    const double delta_yaw = clampValue(yaw_rate - _accelLimitParams.lastYawRate, -max_delta_yaw, max_delta_yaw);

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

bool State_Trotting::isMotionCommandAbove(double eps) const {
    return std::hypot(_motionParams.velocityX, _motionParams.velocityY) > eps ||
           std::fabs(_motionParams.yawRate) > eps;
}

bool State_Trotting::isPureRotationCommand() const {
    const double xy_motion = std::hypot(_motionParams.velocityX, _motionParams.velocityY);
    return xy_motion < _trotParams.pure_rotation_translation_eps &&
           std::fabs(_motionParams.yawRate) > _trotParams.pure_yaw_threshold;
}

const char* State_Trotting::controlModeName() const {
    return _gaitActive ? "active_trot" : "idle_hold";
}

double State_Trotting::gaitRampScale(double now) const {
    const double duration = std::max(0.0, _trotParams.gait_ramp_cycles * _trotParams.cycle_time);
    if (duration <= 1e-6) { return 1.0; }
    return clampValue((now - _gaitStartTime) / duration, 0.0, 1.0);
}

void State_Trotting::startGait(double now) {
    _gaitActive = true;
    _gaitStartTime = now;
    _lastDebugPrintTime = 0.0;
    resetGaitAnchorsToNominal();
    std::cout << "[Trot] gait start: joystick command active, starting trot" << std::endl;
}

void State_Trotting::stopGait(double now) {
    _gaitActive = false;
    _commandActiveSince = -1.0;
    _gaitStartTime = now;
    _lastDebugPrintTime = 0.0;
    _motionParams = {};
    _accelLimitParams = {};
    captureIdleBlendStart(now);
    _ctrlComp->setAllStance();
    std::cout << "[Trot] gait stop: joystick command released, holding stand" << std::endl;
}

double State_Trotting::lateralStabilityFootholdExtra() const {
    const Vec3 rpy = rotMatToRPY(_lowState->getRotMat());
    const Vec3 gyro = _lowState->getGyro();
    const double roll_extra =
        std::max(0.0, _trotParams.lateral_roll_foothold_gain) * std::abs(rpy.x());
    const double gyro_extra =
        std::max(0.0, _trotParams.lateral_gyro_foothold_gain) * std::abs(gyro.x());
    const double vy_extra =
        std::max(0.0, _trotParams.lateral_vy_foothold_gain) *
        std::abs(_motionParams.velocityY);
    return clampValue(
        roll_extra + gyro_extra + vy_extra,
        0.0,
        std::max(0.0, _trotParams.lateral_foothold_extra_limit_m));
}

Vec3 State_Trotting::applyLateralStabilityFoothold(int leg, const Vec3& foothold_in_hip) const {
    Vec3 stable = foothold_in_hip;
    const bool right_leg = (leg == 0 || leg == 2);
    const double side = right_leg ? 1.0 : -1.0;
    const double min_abs_y =
        std::max(0.0, _trotParams.lateral_min_foot_y_m) + lateralStabilityFootholdExtra();
    if (min_abs_y <= 1e-6) {
        return stable;
    }
    if (side > 0.0) {
        stable.y() = std::max(stable.y(), min_abs_y);
    } else {
        stable.y() = std::min(stable.y(), -min_abs_y);
    }
    return stable;
}

Vec3 State_Trotting::clampFootholdToWorkspace(int leg, const Vec3& foothold_in_hip) const {
    const Vec3 shift = foothold_in_hip - _nominalFootPos[leg];
    const Vec3 clamped_shift = clampPlanarShift(
        shift, _trotParams.max_foothold_shift_x, _trotParams.max_foothold_shift_y);
    Vec3 clamped = _nominalFootPos[leg] + clamped_shift;
    clamped = applyLateralStabilityFoothold(leg, clamped);
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
                 asymmetricCycloidLiftOffset(
                     phase, _trotParams.lift_height, _trotParams.swing_lift_peak_phase);
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
                 asymmetricCycloidLiftOffset(
                     phase, _trotParams.lift_height, _trotParams.swing_lift_peak_phase);
    return target;
}

State_Trotting::FootTrajectorySample State_Trotting::computeSwingFootSample(
    int leg, const LegPhaseState& phase_state) const {
    FootTrajectorySample sample;
    const double s = clampValue(phase_state.segmentPhase, 0.0, 1.0);
    const double xy_alpha = cycloidBlend(s);
    const double xy_vel_scale = cycloidBlendVelocityScale(s, phase_state.swingTime);

    const Vec3 start_body = footHipToBodyFrame(leg, _swingStartFootPos[leg]);
    const Vec3 end_body = footHipToBodyFrame(leg, _swingTargetFootPos[leg]);
    const Vec3 delta_body = end_body - start_body;
    const Vec3 target_body = start_body + xy_alpha * delta_body;
    const Vec3 target_vel_body = xy_vel_scale * delta_body;

    sample.pos = footBodyToHipFrame(leg, target_body);
    sample.pos.z() = _swingStartFootPos[leg].z() +
                     (_swingTargetFootPos[leg].z() - _swingStartFootPos[leg].z()) * xy_alpha +
                     asymmetricCycloidLiftOffset(
                         s, _trotParams.lift_height, _trotParams.swing_lift_peak_phase);
    sample.vel = target_vel_body;
    sample.vel.z() = (_swingTargetFootPos[leg].z() - _swingStartFootPos[leg].z()) * xy_vel_scale +
                     asymmetricCycloidLiftVelocity(
                         s,
                         _trotParams.lift_height,
                         phase_state.swingTime,
                         _trotParams.swing_lift_peak_phase);
    return sample;
}

Vec3 State_Trotting::computeStanceFootTarget(int leg, const LegPhaseState& phase_state) const {
    const double s = clampValue(phase_state.segmentPhase, 0.0, 1.0);
    const Vec3 nominal_body = nominalFootBodyPosition(leg);
    const double centered_phase = 0.5 - s;
    const double trajectory_gain = std::max(0.0, _trotParams.stance_trajectory_gain);
    Vec3 target_body = rotateBodyPointYaw(
        nominal_body, trajectory_gain * centered_phase * _motionParams.yawRate * phase_state.stanceTime);
    target_body += trajectory_gain * centered_phase * phase_state.stanceTime *
                   Vec3(_motionParams.velocityX, _motionParams.velocityY, 0.0);
    Vec3 target = footBodyToHipFrame(leg, target_body);
    target.z() = _nominalFootPos[leg].z();
    return clampFootholdToWorkspace(leg, target);
}

Vec3 State_Trotting::computeStanceFootVelocity(int leg, const LegPhaseState& phase_state) const {
    const double s = clampValue(phase_state.segmentPhase, 0.0, 1.0);
    const Vec3 nominal_body = nominalFootBodyPosition(leg);
    const double centered_phase = 0.5 - s;
    const double trajectory_gain = std::max(0.0, _trotParams.stance_trajectory_gain);
    const Vec3 rotated_body = rotateBodyPointYaw(
        nominal_body, trajectory_gain * centered_phase * _motionParams.yawRate * phase_state.stanceTime);

    Vec3 vel_body(-trajectory_gain * _motionParams.velocityX,
                  -trajectory_gain * _motionParams.velocityY,
                  0.0);
    vel_body.x() += trajectory_gain * _motionParams.yawRate * rotated_body.y();
    vel_body.y() += -trajectory_gain * _motionParams.yawRate * rotated_body.x();
    return vel_body;
}

void State_Trotting::updateLegPhaseAnchors(int leg, const LegPhaseState& phase_state) {
    if (phase_state.swing == _prevSwingState[leg]) { return; }

    if (phase_state.swing) {
        const Vec3 q = _lowState->getQ().col(leg);
        _swingStartFootPos[leg] = _robModel->forwardKinematics(q, leg, FrameType::HIP);
        const Vec3 landing_body = computeTouchdownFootBodyTarget(leg, phase_state);
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

void State_Trotting::resetGaitAnchorsToNominal() {
    syncAnchorsForStanding(_nominalFootPos);
}

void State_Trotting::captureIdleBlendStart(double now) {
    _idleBlendStartTime = now;
    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 q = _lowState->getQ().col(leg);
        _idleBlendStartFootPos[leg] =
            _robModel->forwardKinematics(q, leg, FrameType::HIP);
    }
}

void State_Trotting::calculateIKAndApply(int leg, const Vec3& target_foot_in_hip, Vec12& cmd) {
    Vec3 q_des = _robModel->inverseKinematics(target_foot_in_hip, leg, FrameType::HIP);
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

// ===== Force control methods =====

bool State_Trotting::checkStepOrNot() const {
    return (std::fabs(_vCmdBody(0)) > 0.03) ||
           (std::fabs(_vCmdBody(1)) > 0.03) ||
           (std::fabs(_posError(0)) > 0.08) ||
           (std::fabs(_posError(1)) > 0.08) ||
           (std::fabs(_velError(0)) > 0.05) ||
           (std::fabs(_velError(1)) > 0.05) ||
           (std::fabs(_dYawCmd) > 0.20);
}

void State_Trotting::calcBodyWrench() {
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    // User commands in body frame
    _vCmdBody(0) = _motionParams.velocityX;
    _vCmdBody(1) = _motionParams.velocityY;
    _vCmdBody(2) = 0.0;
    _dYawCmd = _motionParams.yawRate;
    _dYawCmd = 0.9 * _dYawCmdPast + 0.1 * _dYawCmd;
    _dYawCmdPast = _dYawCmd;

    // Convert to global frame
    _vCmdGlobal = _G2B_RotMat.transpose() * _vCmdBody;
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0) - 0.2, _velBody(0) + 0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1) - 0.2, _velBody(1) + 0.2));
    _vCmdGlobal(2) = 0.0;

    // Integrate desired position
    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    // Integrate desired yaw
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;
    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;

    // Body PD controller
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd = _Kpw * rotMatToExp(_Rd * _G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    const auto& f = _ctrlComp->parameters.force;
    // Saturate for safety
    _ddPcd(0) = saturation(_ddPcd(0), f.acc_xy_sat);
    _ddPcd(1) = saturation(_ddPcd(1), f.acc_xy_sat);
    _ddPcd(2) = saturation(_ddPcd(2), f.acc_z_sat);
    _dWbd(0) = saturation(_dWbd(0), f.w_roll_pitch_sat);
    _dWbd(1) = saturation(_dWbd(1), f.w_roll_pitch_sat);
    _dWbd(2) = saturation(_dWbd(2), f.w_yaw_sat);
}

void State_Trotting::calcFootForces() {
    const double masterT = getTimeSec() - _startTime;

    // Build contact flags from gait scheduler
    VecInt4 contact = VecInt4::Zero();
    Vec4 phase = Vec4::Constant(0.5);
    std::array<Vec3, 4> footTargetHip;
    footTargetHip.fill(Vec3::Zero());

    for (int leg = 0; leg < 4; ++leg) {
        const LegPhaseState phase_state = computeLegPhaseState(leg, masterT);
        updateLegPhaseAnchors(leg, phase_state);
        contact(leg) = phase_state.swing ? 0 : 1;
        phase(leg) = phase_state.segmentPhase;

        footTargetHip[leg] = phase_state.swing
            ? computeSwingFootTarget(leg, phase_state.segmentPhase)
            : computeStanceFootTarget(leg, phase_state);
    }

    // Convert hip-frame targets to global frame for swing leg PD
    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 target_body = footHipToBodyFrame(leg, footTargetHip[leg]);
        _posFeetGlobalGoal.col(leg) = _posBody + _B2G_RotMat * target_body;
        // Simple velocity estimate for swing legs
        _velFeetGlobalGoal.col(leg).setZero();
    }

    // Compute stance forces via QP
    // calF returns ground-to-foot force (positive = upward). Pass directly to getTau.
    _forceFeetGlobal = _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, contact);

    // Override swing leg forces with trajectory tracking PD
    for (int leg = 0; leg < 4; ++leg) {
        if (contact(leg) == 0) {
            _forceFeetGlobal.col(leg) =
                _KpSwing * (_posFeetGlobalGoal.col(leg) - _posFeetGlobal.col(leg)) +
                _KdSwing * (_velFeetGlobalGoal.col(leg) - _velFeetGlobal.col(leg));
        }
    }

    // Rotate to body frame
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    // Update contact/phase in context
    _ctrlComp->setContactPhase(contact, phase);
}

void State_Trotting::calcVmcFootForces(const VecInt4& contact, double force_scale) {
    _forceFeetBody.setZero();

    int contact_count = 0;
    for (int leg = 0; leg < 4; ++leg) {
        contact_count += contact(leg) != 0 ? 1 : 0;
    }
    contact_count = std::max(1, contact_count);
    const bool active_two_leg_support = contact_count <= 2;
    const Vec3 foot_kp = active_two_leg_support ? _trotParams.active_vmc_kp_foot
                                                : _trotParams.vmc_kp_foot;
    const Vec3 foot_kd = active_two_leg_support ? _trotParams.active_vmc_kd_foot
                                                : _trotParams.vmc_kd_foot;
    const Vec3 error_limit = active_two_leg_support ? _trotParams.active_vmc_error_limit_m
                                                    : _trotParams.vmc_error_limit_m;

    const double gravity_per_leg = _robModel->getRobMass() * 9.81 / static_cast<double>(contact_count);
    const double friction_ratio = std::max(0.0, _ctrlComp->parameters.force.friction_ratio);
    const double max_force_delta =
        std::max(1.0, _trotParams.vmc_force_rate_limit_n_per_s) * _ctrlComp->dt;
    const double fz_min = active_two_leg_support ? _trotParams.active_fz_min_per_leg_n
                                                 : _trotParams.idle_fz_min_per_leg_n;
    const double fz_max = active_two_leg_support ? _trotParams.active_fz_max_per_leg_n
                                                 : _trotParams.idle_fz_max_per_leg_n;

    const auto& hybrid = _ctrlComp->parameters.hybrid_stand;
    const Vec3 cur_rpy = rotMatToRPY(_lowState->getRotMat());
    const Vec3 gyro_body = _lowState->getGyro();
    const double attitude_blend = clampValue(force_scale, 0.0, 1.0);
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
    const double attitude_fz_gain = std::max(0.0, _trotParams.vmc_attitude_fz_gain);
    const double attitude_fz_limit = std::max(0.0, _trotParams.vmc_attitude_fz_limit_n);

    for (int leg = 0; leg < 4; ++leg) {
        if (contact(leg) == 0) {
            _lastForceFeetBody.col(leg).setZero();
            continue;
        }

        const Vec3 q_leg = _lowState->getQ().col(leg);
        const Vec3 foot_now = _robModel->forwardKinematics(q_leg, leg, FrameType::HIP);
        const Vec3 foot_target = footBodyToHipFrame(leg, _posFeet2BGoal.col(leg));
        const Vec3 foot_vel = _robModel->getFootVelocity(*_lowState, leg);
        const Vec3 foot_vel_target = _velFeet2BGoal.col(leg);

        Vec3 foot_error = foot_target - foot_now;
        for (int axis = 0; axis < 3; ++axis) {
            const double limit = std::abs(error_limit(axis));
            foot_error(axis) = clampValue(foot_error(axis), -limit, limit);
        }

        Vec3 foot_force =
            -foot_kp.cwiseProduct(foot_error) +
            foot_kd.cwiseProduct(foot_vel - foot_vel_target);
        foot_force.z() += gravity_per_leg;

        const bool right_leg = (leg == 0 || leg == 2);
        const bool front_leg = (leg == 0 || leg == 1);
        const double roll_delta = right_leg ? roll_force_unit : -roll_force_unit;
        const double pitch_delta = front_leg ? pitch_force_unit : -pitch_force_unit;
        foot_force.z() += clampValue(
            attitude_fz_gain * (roll_delta + pitch_delta),
            -attitude_fz_limit,
            attitude_fz_limit);
        if (active_two_leg_support && !front_leg) {
            foot_force.z() += std::max(0.0, _trotParams.active_rear_fz_boost_n);
        }

        foot_force.z() = std::max(std::max(0.0, fz_min), foot_force.z());
        if (fz_max > 0.0) {
            foot_force.z() = std::min(foot_force.z(), fz_max);
        }
        const double fxy_limit = friction_ratio * foot_force.z();
        foot_force.x() = clampValue(foot_force.x(), -fxy_limit, fxy_limit);
        foot_force.y() = clampValue(foot_force.y(), -fxy_limit, fxy_limit);

        Vec3 limited_force = foot_force;
        if (_forceRateInitialized) {
            limited_force = _lastForceFeetBody.col(leg);
            if (limited_force.z() < fz_min && foot_force.z() >= fz_min) {
                limited_force.z() = fz_min;
            }
            for (int axis = 0; axis < 3; ++axis) {
                limited_force(axis) += clampValue(
                    foot_force(axis) - limited_force(axis), -max_force_delta, max_force_delta);
            }
        }
        limited_force.z() = std::max(std::max(0.0, fz_min), limited_force.z());
        if (fz_max > 0.0) {
            limited_force.z() = std::min(limited_force.z(), fz_max);
        }
        const double limited_fxy = friction_ratio * limited_force.z();
        limited_force.x() = clampValue(limited_force.x(), -limited_fxy, limited_fxy);
        limited_force.y() = clampValue(limited_force.y(), -limited_fxy, limited_fxy);
        _forceFeetBody.col(leg) = limited_force;
        _lastForceFeetBody.col(leg) = limited_force;
    }
    if (active_two_leg_support && contact_count == 2) {
        int stance_a = -1;
        int stance_b = -1;
        for (int leg = 0; leg < 4; ++leg) {
            if (contact(leg) == 0) {
                continue;
            }
            if (stance_a < 0) {
                stance_a = leg;
            } else {
                stance_b = leg;
            }
        }

        if (stance_a >= 0 && stance_b >= 0) {
            const double balance_gain =
                clampValue(_trotParams.active_diagonal_fz_balance_gain, 0.0, 1.0);
            const double avg_fz =
                0.5 * (_forceFeetBody(2, stance_a) + _forceFeetBody(2, stance_b));
            for (int leg : {stance_a, stance_b}) {
                Vec3 balanced_force = _forceFeetBody.col(leg);
                balanced_force.z() += balance_gain * (avg_fz - balanced_force.z());
                balanced_force.z() = std::max(std::max(0.0, fz_min), balanced_force.z());
                if (fz_max > 0.0) {
                    balanced_force.z() = std::min(balanced_force.z(), fz_max);
                }
                const double balanced_fxy = friction_ratio * balanced_force.z();
                balanced_force.x() = clampValue(balanced_force.x(), -balanced_fxy, balanced_fxy);
                balanced_force.y() = clampValue(balanced_force.y(), -balanced_fxy, balanced_fxy);
                _forceFeetBody.col(leg) = balanced_force;
                _lastForceFeetBody.col(leg) = balanced_force;
            }
        }
    }
    _forceRateInitialized = true;

    _forceFeetGlobal = _B2G_RotMat * _forceFeetBody;
}

void State_Trotting::calcPlanarVmcTorques() {
    _forceFeetBody.setZero();

    const Vec34 q_legs = _lowState->getQ();
    const Vec12 q = vec34ToVec12(q_legs);
    const double kp_x = std::max(0.0, _trotParams.planar_vmc_kp_x);
    const double kd_x = std::max(0.0, _trotParams.planar_vmc_kd_x);
    const double kp_z = std::max(0.0, _trotParams.planar_vmc_kp_z);
    const double kd_z = std::max(0.0, _trotParams.planar_vmc_kd_z);
    const double err_x_limit = std::abs(_trotParams.planar_vmc_error_limit_x_m);
    const double err_z_limit = std::abs(_trotParams.planar_vmc_error_limit_z_m);
    const double tau_limit = std::abs(_trotParams.planar_vmc_tau_limit_nm);

    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 q_leg = q_legs.col(leg);
        const Vec3 foot_now = _robModel->forwardKinematics(q_leg, leg, FrameType::HIP);
        const Vec3 foot_target = footBodyToHipFrame(leg, _posFeet2BGoal.col(leg));
        const Vec3 foot_vel = _robModel->getFootVelocity(*_lowState, leg);
        const Vec3 foot_vel_target = _velFeet2BGoal.col(leg);

        const double err_x = clampValue(foot_target.x() - foot_now.x(),
                                        -err_x_limit,
                                        err_x_limit);
        const double err_z = clampValue(foot_target.z() - foot_now.z(),
                                        -err_z_limit,
                                        err_z_limit);

        // 与 FixedStand 的力方向约定保持一致：正 z 为支撑方向，随后统一用 -J^T F 变换到关节。
        Vec3 foot_force = Vec3::Zero();
        foot_force.x() = -kp_x * err_x + kd_x * (foot_vel.x() - foot_vel_target.x());
        foot_force.z() = -kp_z * err_z + kd_z * (foot_vel.z() - foot_vel_target.z());
        _forceFeetBody.col(leg) = foot_force;
    }

    _forceFeetGlobal = _B2G_RotMat * _forceFeetBody;
    _tau = -_robModel->getTau(q, _forceFeetBody);

    for (int leg = 0; leg < 4; ++leg) {
        const int id = leg * 3;
        _tau(id) = 0.0;  // q0/hip 外展只走位置环，不吃 VMC 前馈力矩。
        _tau(id + 1) = clampValue(_tau(id + 1), -tau_limit, tau_limit);
        _tau(id + 2) = clampValue(_tau(id + 2), -tau_limit, tau_limit);
    }
}

void State_Trotting::calcJointTorques() {
    Vec12 q = vec34ToVec12(_lowState->getQ());
    _tau = -_robModel->getTau(q, _forceFeetBody);

    // 摆动腿不用力矩前馈，只保留位置环
    int contact_count = 0;
    for (int leg = 0; leg < 4; ++leg) {
        contact_count += _ctrlComp->contact(leg) != 0 ? 1 : 0;
    }
    const double q0_tau_limit =
        std::abs(contact_count <= 2 ? _trotParams.active_vmc_q0_tau_limit_nm
                                    : _trotParams.vmc_q0_tau_limit_nm);
    for (int leg = 0; leg < 4; ++leg) {
        if (_ctrlComp->contact(leg) == 0) {
            _tau.segment(leg * 3, 3).setZero();
        } else {
            _tau(leg * 3) = clampValue(_tau(leg * 3), -q0_tau_limit, q0_tau_limit);
        }
    }
}

void State_Trotting::calcSwingQQd() {
    // 复用 calcFootForces() 已设置的 _ctrlComp->contact
    const VecInt4& contact = _ctrlComp->contact;

    // Compute body-frame foot positions/velocities for IK
    Vec34 posFeet2B = _robModel->getFeet2BPositions(*_lowState, FrameType::BODY);

    for (int leg = 0; leg < 4; ++leg) {
        _posFeet2BGoal.col(leg) = _G2B_RotMat * (_posFeetGlobalGoal.col(leg) - _posBody);
        _velFeet2BGoal.col(leg) = _G2B_RotMat * (_velFeetGlobalGoal.col(leg) - _velBody);
    }

    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(posFeet2B, _velFeet2BGoal, FrameType::BODY));
}

void State_Trotting::buildJointGoalsFromBodyTargets() {
    Vec34 posFeet2B = _robModel->getFeet2BPositions(*_lowState, FrameType::BODY);
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(posFeet2B, _velFeet2BGoal, FrameType::BODY));

    for (int leg = 0; leg < 4; ++leg) {
        _qGoal.col(leg) = clampJointAngles(_qGoal.col(leg));
        _lastLegQ[leg] = _qGoal.col(leg);
    }
}

void State_Trotting::sendHybridCommands(const VecInt4& contact) {
    int contact_count = 0;
    for (int leg = 0; leg < 4; ++leg) {
        contact_count += contact(leg) != 0 ? 1 : 0;
    }
    const bool active_two_leg_support = contact_count <= 2;

    for (int leg = 0; leg < 4; ++leg) {
        const Vec3 kp = (contact(leg) == 0) ? _trotParams.swing_joint_kp
            : (active_two_leg_support ? _trotParams.active_stance_joint_kp
                                      : _trotParams.stance_joint_kp);
        const Vec3 kd = (contact(leg) == 0) ? _trotParams.swing_joint_kd
            : (active_two_leg_support ? _trotParams.active_stance_joint_kd
                                      : _trotParams.stance_joint_kd);
        for (int joint = 0; joint < 3; ++joint) {
            const int id = leg * 3 + joint;
            _lowCmd->motorCmd[id].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            _lowCmd->motorCmd[id].q = _qGoal(joint, leg);
            _lowCmd->motorCmd[id].dq = _qdGoal(joint, leg);
            _lowCmd->motorCmd[id].Kp = kp(joint);
            _lowCmd->motorCmd[id].Kd = kd(joint);
            _lowCmd->motorCmd[id].tau = _tau(id);
        }
    }
}

void State_Trotting::sendPlanarVmcCommands(const VecInt4& contact) {
    const double q0_kp = std::max(0.0, _trotParams.planar_q0_kp);
    const double q0_kd = std::max(0.0, _trotParams.planar_q0_kd);
    int contact_count = 0;
    for (int leg = 0; leg < 4; ++leg) {
        contact_count += contact(leg) != 0 ? 1 : 0;
    }
    const bool active_two_leg_support = contact_count <= 2;

    for (int leg = 0; leg < 4; ++leg) {
        const int base = leg * 3;
        const Vec3 kp = (contact(leg) == 0) ? _trotParams.swing_joint_kp
            : (active_two_leg_support ? _trotParams.active_stance_joint_kp
                                      : _trotParams.stance_joint_kp);
        const Vec3 kd = (contact(leg) == 0) ? _trotParams.swing_joint_kd
            : (active_two_leg_support ? _trotParams.active_stance_joint_kd
                                      : _trotParams.stance_joint_kd);

        _lowCmd->motorCmd[base].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
        _lowCmd->motorCmd[base].q = _qGoal(0, leg);
        _lowCmd->motorCmd[base].dq = _qdGoal(0, leg);
        _lowCmd->motorCmd[base].Kp = q0_kp;
        _lowCmd->motorCmd[base].Kd = q0_kd;
        _lowCmd->motorCmd[base].tau = 0.0;

        for (int joint = 1; joint < 3; ++joint) {
            const int id = base + joint;
            _lowCmd->motorCmd[id].mode = static_cast<unsigned int>(ControlMode::COMPOUND);
            _lowCmd->motorCmd[id].q = _qGoal(joint, leg);
            _lowCmd->motorCmd[id].dq = _qdGoal(joint, leg);
            _lowCmd->motorCmd[id].Kp = kp(joint);
            _lowCmd->motorCmd[id].Kd = kd(joint);
            _lowCmd->motorCmd[id].tau = _tau(id);
        }
    }
}

void State_Trotting::printTrotDebug(double now,
                                    const VecInt4& contact,
                                    const Vec4& phase,
                                    double force_scale) {
    const double period = std::max(0.02, _trotParams.debug_print_period_s);
    if (now - _lastDebugPrintTime < period) {
        return;
    }
    _lastDebugPrintTime = now;

    Vec3 foot_err_fr = Vec3::Zero();
    Vec4 foot_err_z = Vec4::Zero();
    if (_posFeet2BGoal.cols() > 0) {
        const Vec3 fr_now = _robModel->forwardKinematics(
            _lowState->getQ().col(0), 0, FrameType::HIP);
        foot_err_fr = footBodyToHipFrame(0, _posFeet2BGoal.col(0)) - fr_now;
        for (int leg = 0; leg < 4; ++leg) {
            const Vec3 foot_now = _robModel->forwardKinematics(
                _lowState->getQ().col(leg), leg, FrameType::HIP);
            const Vec3 foot_err =
                footBodyToHipFrame(leg, _posFeet2BGoal.col(leg)) - foot_now;
            foot_err_z(leg) = foot_err.z();
        }
    }

    const Vec3 rpy = rotMatToRPY(_lowState->getRotMat());
    std::cout << std::fixed << std::setprecision(3)
              << "[Trot][dbg] mode=" << controlModeName()
              << " forceMode=" << _trotParams.force_mode
              << " forceScale=" << force_scale
              << " cmdVel=" << _vCmdBody.transpose()
              << " bodyVel=" << (_G2B_RotMat * _velBody).transpose()
              << " contact=" << contact.transpose()
              << " phase=" << phase.transpose()
              << " footErr_FR=" << foot_err_fr.transpose()
              << " footErrZ=" << foot_err_z.transpose()
              << " Fz=" << _forceFeetBody.row(2)
              << " tau_FR=" << _tau.segment(0, 3).transpose()
              << " tau_FL=" << _tau.segment(3, 3).transpose()
              << " tau_RR=" << _tau.segment(6, 3).transpose()
              << " tau_RL=" << _tau.segment(9, 3).transpose()
              << " rpy=" << rpy.transpose()
              << std::endl;
}

void State_Trotting::applyTorqueSafety(const VecInt4& contact) {
    const auto& f = _ctrlComp->parameters.force;
    if (!_torqueSafetyInitialized) {
        for (int i = 0; i < 12; ++i) {
            _tau(i) = clampValue(_tau(i), -f.tau_limit, f.tau_limit);
        }
        _tauPrev = _tau;
        _torqueSafetyInitialized = true;
        return;
    }

    int contact_count = 0;
    for (int leg = 0; leg < 4; ++leg) {
        contact_count += contact(leg) != 0 ? 1 : 0;
    }
    const bool active_two_leg_support = contact_count <= 2;
    const double active_tau_rate_limit =
        std::max(f.tau_rate_limit, _trotParams.active_tau_rate_limit_nm_per_s);
    const double default_max_delta = f.tau_rate_limit * _ctrlComp->dt;
    const double active_max_delta = active_tau_rate_limit * _ctrlComp->dt;

    for (int i = 0; i < 12; ++i) {
        const int leg = i / 3;
        const double max_delta =
            (active_two_leg_support && contact(leg) != 0) ? active_max_delta : default_max_delta;
        double clamped = clampValue(_tau(i), -f.tau_limit, f.tau_limit);
        double delta = clampValue(clamped - _tauPrev(i), -max_delta, max_delta);
        _tau(i) = clampValue(_tauPrev(i) + delta, -f.tau_limit, f.tau_limit);
    }
    _tauPrev = _tau;
}

void State_Trotting::zeroSwingLegTorques(const VecInt4& contact) {
    for (int leg = 0; leg < 4; ++leg) {
        if (contact(leg) == 0) {
            const int base = leg * 3;
            _tau.segment(base, 3).setZero();
            _tauPrev.segment(base, 3).setZero();
        }
    }
}

void State_Trotting::runIdleHold() {
    const double now = getTimeSec();
    VecInt4 contact = VecInt4::Ones();
    Vec4 phase = Vec4::Constant(0.5);
    _ctrlComp->setContactPhase(contact, phase);

    const double settle_duration = std::max(0.0, _trotParams.idle_settle_duration_s);
    const double settle_phase = (settle_duration <= 1e-6)
        ? 1.0
        : clampValue((now - _idleBlendStartTime) / settle_duration, 0.0, 1.0);
    const double settle_blend = cycloidBlend(settle_phase);
    std::array<Vec3, 4> idle_targets;

    for (int leg = 0; leg < 4; ++leg) {
        idle_targets[leg] =
            _idleBlendStartFootPos[leg] +
            settle_blend * (_nominalFootPos[leg] - _idleBlendStartFootPos[leg]);
        if (settle_phase >= 1.0) {
            idle_targets[leg] = _nominalFootPos[leg];
        }
        _posFeet2BGoal.col(leg) = footHipToBodyFrame(leg, idle_targets[leg]);
        _velFeet2BGoal.col(leg).setZero();
    }
    syncAnchorsForStanding(idle_targets);

    buildJointGoalsFromBodyTargets();
    if (_trotParams.force_mode == "planar_vmc") {
        calcPlanarVmcTorques();
    } else {
        calcVmcFootForces(contact, 1.0);
        calcJointTorques();
    }
    applyTorqueSafety(contact);
    if (_trotParams.force_mode == "planar_vmc") {
        sendPlanarVmcCommands(contact);
    } else {
        sendHybridCommands(contact);
    }
    printTrotDebug(now, contact, phase, 1.0);
}

void State_Trotting::runVmcTrot(double now) {
    const double ramp_duration = std::max(0.0, _trotParams.gait_ramp_cycles * _trotParams.cycle_time);
    if (now - _gaitStartTime < ramp_duration) {
        runIdleHold();
        return;
    }

    const double masterT = now - _gaitStartTime - ramp_duration;
    VecInt4 contact = VecInt4::Zero();
    Vec4 phase = Vec4::Constant(0.5);

    for (int leg = 0; leg < 4; ++leg) {
        const LegPhaseState phase_state = computeLegPhaseState(leg, masterT);
        updateLegPhaseAnchors(leg, phase_state);
        contact(leg) = phase_state.swing ? 0 : 1;
        phase(leg) = phase_state.segmentPhase;

        if (phase_state.swing) {
            const FootTrajectorySample sample = computeSwingFootSample(leg, phase_state);
            _posFeet2BGoal.col(leg) = footHipToBodyFrame(leg, sample.pos);
            _velFeet2BGoal.col(leg) = sample.vel;
        } else {
            const Vec3 stance_pos = computeStanceFootTarget(leg, phase_state);
            _posFeet2BGoal.col(leg) = footHipToBodyFrame(leg, stance_pos);
            _velFeet2BGoal.col(leg) = computeStanceFootVelocity(leg, phase_state);
        }
    }

    _ctrlComp->setContactPhase(contact, phase);
    buildJointGoalsFromBodyTargets();
    calcVmcFootForces(contact, 1.0);
    calcJointTorques();
    applyTorqueSafety(contact);
    zeroSwingLegTorques(contact);
    sendHybridCommands(contact);
    printTrotDebug(now, contact, phase, 1.0);
}

void State_Trotting::runPlanarVmcTrot(double now) {
    const double ramp_duration = std::max(0.0, _trotParams.gait_ramp_cycles * _trotParams.cycle_time);
    if (now - _gaitStartTime < ramp_duration) {
        runIdleHold();
        return;
    }

    const double masterT = now - _gaitStartTime - ramp_duration;
    VecInt4 contact = VecInt4::Zero();
    Vec4 phase = Vec4::Constant(0.5);

    for (int leg = 0; leg < 4; ++leg) {
        const LegPhaseState phase_state = computeLegPhaseState(leg, masterT);
        updateLegPhaseAnchors(leg, phase_state);
        contact(leg) = phase_state.swing ? 0 : 1;
        phase(leg) = phase_state.segmentPhase;

        if (phase_state.swing) {
            const FootTrajectorySample sample = computeSwingFootSample(leg, phase_state);
            _posFeet2BGoal.col(leg) = footHipToBodyFrame(leg, sample.pos);
            _velFeet2BGoal.col(leg) = sample.vel;
        } else {
            const Vec3 stance_pos = computeStanceFootTarget(leg, phase_state);
            _posFeet2BGoal.col(leg) = footHipToBodyFrame(leg, stance_pos);
            _velFeet2BGoal.col(leg) = computeStanceFootVelocity(leg, phase_state);
        }
    }

    _ctrlComp->setContactPhase(contact, phase);
    buildJointGoalsFromBodyTargets();
    calcPlanarVmcTorques();
    applyTorqueSafety(contact);
    zeroSwingLegTorques(contact);
    sendPlanarVmcCommands(contact);
    printTrotDebug(now, contact, phase, 1.0);
}

void State_Trotting::runQpTrot(double trans) {
    calcFootForces();
    calcJointTorques();
    calcSwingQQd();
    const VecInt4& contact = _ctrlComp->contact;
    applyTorqueSafety(contact);
    _tau *= trans;

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for (int leg = 0; leg < 4; ++leg) {
        if (contact(leg) == 0) {
            _lowCmd->setSwingGain(leg);
        } else {
            _lowCmd->setStableGain(leg);
        }
    }
}

// ===== Main run loop =====

void State_Trotting::run() {
    const double transition_steps = _ctrlComp->parameters.force.transition_steps;
    if (_transitionCount < static_cast<int>(transition_steps)) {
        ++_transitionCount;
    }
    const double trans = std::min(1.0, static_cast<double>(_transitionCount) / transition_steps);

    processJoystickInput();
    calcBodyWrench();

    const double now = getTimeSec();
    const bool joystick_active =
        std::fabs(static_cast<double>(_lowState->userValue.lx)) > _joyMapping.deadband ||
        std::fabs(static_cast<double>(_lowState->userValue.ly)) > _joyMapping.deadband ||
        std::fabs(static_cast<double>(_lowState->userValue.rx)) > _joyMapping.deadband;
    const bool start_command =
        joystick_active && isMotionCommandAbove(_trotParams.gait_start_command_eps);
    const bool keep_command = isMotionCommandAbove(_trotParams.gait_stop_command_eps);

    if (!_gaitActive) {
        if (!start_command) {
            _commandActiveSince = -1.0;
            runIdleHold();
            return;
        }
        if (_commandActiveSince < 0.0) {
            _commandActiveSince = now;
        }
        if (now - _commandActiveSince < _trotParams.gait_start_debounce_s) {
            runIdleHold();
            return;
        }
        startGait(now);
    } else if (!keep_command) {
        stopGait(now);
        runIdleHold();
        return;
    }

    if (_trotParams.force_mode == "qp") {
        runQpTrot(trans);
    } else if (_trotParams.force_mode == "planar_vmc") {
        runPlanarVmcTrot(now);
    } else {
        runVmcTrot(now);
    }
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
    _tauPrev.setZero();
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
