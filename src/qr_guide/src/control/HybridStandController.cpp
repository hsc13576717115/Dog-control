#include "control/HybridStandController.h"

#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/SVD>

#include "common/mathTools.h"
#include "common/unitreeRobot.h"
#include "control/Estimator.h"
#include "message/LowlevelState.h"

namespace qr_guide {

namespace {

double clampValue(double value, double min_value, double max_value) {
    return std::min(std::max(value, min_value), max_value);
}

double sanitizeLowerLimit(double a, double b) {
    return std::min(a, b);
}

double sanitizeUpperLimit(double a, double b) {
    return std::max(a, b);
}

}  // namespace

HybridStandController::HybridStandController(const HybridStandParameters& parameters, double dt)
    : parameters_(parameters),
      dt_(dt) {
    reset();
}

void HybridStandController::reset() {
    last_tau_command_.setZero();
    debug_snapshot_ = DebugSnapshot{};
}

bool HybridStandController::computeTorqueFeedforward(
    const std::array<Vec3, NumLeg>& target_feet_in_body,
    LowlevelState& low_state,
    Estimator* estimator,
    QuadrupedRobot& robot_model,
    double transition_scale,
    Vec12* tau_command) {
    if (tau_command == nullptr || estimator == nullptr) {
        debug_snapshot_.status = DebugStatus::INVALID_STATE;
        return false;
    }

    tau_command->setZero();
    debug_snapshot_ = DebugSnapshot{};
    if (hasMotorFault(low_state)) {
        debug_snapshot_.status = DebugStatus::MOTOR_FAULT;
        return false;
    }

    Vec3 body_task_wrench = Vec3::Zero();
    if (!computeBodyTaskWrench(
            target_feet_in_body, low_state, estimator, robot_model, &body_task_wrench, &debug_snapshot_)) {
        debug_snapshot_.status = DebugStatus::INVALID_STATE;
        return false;
    }
    debug_snapshot_.body_task_wrench = body_task_wrench;

    const Vec34 current_feet_in_body = robot_model.getFeet2BPositions(low_state, FrameType::BODY);
    if (!current_feet_in_body.allFinite()) {
        debug_snapshot_.status = DebugStatus::INVALID_FEET;
        return false;
    }

    Vec4 vertical_forces = Vec4::Zero();
    if (!allocateVerticalForces(current_feet_in_body, body_task_wrench, &vertical_forces)) {
        debug_snapshot_.status = DebugStatus::ALLOCATION_FAIL;
        return false;
    }
    debug_snapshot_.vertical_forces = vertical_forces;

    Vec34 foot_forces = Vec34::Zero();
    for (int leg = 0; leg < NumLeg; ++leg) {
        foot_forces.col(leg) << 0.0, 0.0, vertical_forces(leg);
    }

    const Vec12 current_q = vec34ToVec12(low_state.getQ());
    Vec12 tau_desired = robot_model.getTau(current_q, foot_forces);
    if (!tau_desired.allFinite()) {
        debug_snapshot_.status = DebugStatus::INVALID_TORQUE;
        return false;
    }

    tau_desired *= clampValue(transition_scale, 0.0, 1.0);
    *tau_command = applyJointTorqueSafety(tau_desired);
    if (!tau_command->allFinite()) {
        debug_snapshot_.status = DebugStatus::INVALID_TORQUE;
        return false;
    }

    debug_snapshot_.status = DebugStatus::SUCCESS;
    return true;
}

bool HybridStandController::hasMotorFault(const LowlevelState& low_state) const {
    for (int index = 0; index < 12; ++index) {
        if (low_state.motorState[index].fault != 0) {
            return true;
        }
    }
    return false;
}

bool HybridStandController::computeBodyTaskWrench(
    const std::array<Vec3, NumLeg>& target_feet_in_body,
    LowlevelState& low_state,
    Estimator* estimator,
    QuadrupedRobot& robot_model,
    Vec3* body_task_wrench,
    DebugSnapshot* debug_snapshot) const {
    if (body_task_wrench == nullptr || estimator == nullptr || debug_snapshot == nullptr) {
        return false;
    }

    const Vec3 rpy = rotMatToRPY(low_state.getRotMat());
    const Vec3 gyro = low_state.getGyro();
    const Vec3 position = estimator->getPosition();
    const Vec3 velocity = estimator->getVelocity();
    debug_snapshot->rpy = rpy;
    debug_snapshot->gyro = gyro;
    debug_snapshot->position = position;
    debug_snapshot->velocity = velocity;
    if (!rpy.allFinite() || !gyro.allFinite() || !position.allFinite() || !velocity.allFinite()) {
        return false;
    }

    double z_des = 0.0;
    for (int leg = 0; leg < NumLeg; ++leg) {
        z_des += -target_feet_in_body[leg].z();
    }
    z_des = z_des / static_cast<double>(NumLeg) + robot_model.getParameters().foot_radius_m;
    debug_snapshot->z_des = z_des;

    const double mass = robot_model.getParameters().mass.total_mass_kg;
    body_task_wrench->x() =
        mass * 9.81 + parameters_.kp_z * (z_des - position.z()) + parameters_.kd_z * (0.0 - velocity.z());
    body_task_wrench->y() =
        parameters_.kp_roll * (0.0 - rpy.x()) + parameters_.kd_roll * (0.0 - gyro.x());
    body_task_wrench->z() =
        parameters_.kp_pitch * (0.0 - rpy.y()) + parameters_.kd_pitch * (0.0 - gyro.y());
    return body_task_wrench->allFinite();
}

bool HybridStandController::allocateVerticalForces(const Vec34& current_feet_in_body,
                                                   const Vec3& body_task_wrench,
                                                   Vec4* vertical_forces) const {
    if (vertical_forces == nullptr) {
        return false;
    }

    Eigen::Matrix<double, 3, 4> allocation = Eigen::Matrix<double, 3, 4>::Zero();
    for (int leg = 0; leg < NumLeg; ++leg) {
        const double x = current_feet_in_body(0, leg);
        const double y = current_feet_in_body(1, leg);
        allocation(0, leg) = 1.0;
        allocation(1, leg) = y;
        allocation(2, leg) = -x;
    }

    const Mat3 aat = allocation * allocation.transpose();
    if (!aat.allFinite()) {
        return false;
    }

    Eigen::JacobiSVD<Mat3> svd(aat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Vec3 singular_values = svd.singularValues();
    if (!singular_values.allFinite() || singular_values(0) < 1e-9 ||
        singular_values(2) / singular_values(0) < 1e-6) {
        return false;
    }

    const Vec3 lambda = svd.solve(body_task_wrench);
    if (!lambda.allFinite()) {
        return false;
    }

    Vec4 solved_forces = allocation.transpose() * lambda;
    if (!solved_forces.allFinite()) {
        return false;
    }

    const double fz_min = sanitizeLowerLimit(parameters_.fz_min_per_leg_n, parameters_.fz_max_per_leg_n);
    const double fz_max = sanitizeUpperLimit(parameters_.fz_min_per_leg_n, parameters_.fz_max_per_leg_n);
    for (int leg = 0; leg < NumLeg; ++leg) {
        solved_forces(leg) = clampValue(solved_forces(leg), fz_min, fz_max);
    }

    *vertical_forces = solved_forces;
    return true;
}

Vec12 HybridStandController::applyJointTorqueSafety(const Vec12& tau_desired) {
    Vec12 limited = Vec12::Zero();
    const double tau_limit = std::abs(parameters_.tau_limit_nm);
    const double max_delta = std::abs(parameters_.tau_rate_limit_nm_per_s) * std::max(dt_, 1e-4);

    for (int index = 0; index < 12; ++index) {
        const double clamped_tau = clampValue(tau_desired(index), -tau_limit, tau_limit);
        const double delta = clampValue(clamped_tau - last_tau_command_(index), -max_delta, max_delta);
        limited(index) = clampValue(last_tau_command_(index) + delta, -tau_limit, tau_limit);
    }

    last_tau_command_ = limited;
    return limited;
}

}  // namespace qr_guide
