#include "model/RobotModel.h"

#include <iostream>

#include "common/mathTools.h"

namespace qr_guide {

RobotModel::RobotModel(const RobotParameters& parameters)
    : parameters_(parameters) {
    for (int leg = 0; leg < NumLeg; ++leg) {
        // 每条腿都共享同一套数学形式，只是长度和髋安装点来自统一参数。
        legs_[leg] = std::make_unique<LegKinematics>(
            leg,
            parameters_.l0,
            parameters_.l1,
            parameters_.l2,
            parameters_.hip_mounts_in_body[leg]);
    }
}

Vec3 RobotModel::footPosition(const Vec3& q, int leg_id, FrameType frame) const {
    if (frame == FrameType::BODY) {
        return legs_[leg_id]->footInBody(q);
    }
    if (frame == FrameType::HIP) {
        return legs_[leg_id]->footInHip(q);
    }
    std::cerr << "[RobotModel] footPosition only supports BODY or HIP frame." << std::endl;
    std::exit(-1);
}

Vec3 RobotModel::jointAngles(const Vec3& foot_position, int leg_id, FrameType frame) const {
    return legs_[leg_id]->jointAnglesFromFoot(foot_position, frame);
}

Mat3 RobotModel::jacobian(const Vec3& q, int leg_id) const {
    return legs_[leg_id]->jacobian(q);
}

Vec3 RobotModel::footVelocity(const Vec3& q, const Vec3& qd, int leg_id) const {
    return legs_[leg_id]->footVelocityInHip(q, qd);
}

Vec3 RobotModel::jointVelocity(const Vec3& q, const Vec3& foot_velocity, int leg_id) const {
    return legs_[leg_id]->jointVelocityFromFootVelocity(q, foot_velocity);
}

Vec3 RobotModel::jointTorques(const Vec3& q, const Vec3& foot_force, int leg_id) const {
    return legs_[leg_id]->jointTorquesFromFootForce(q, foot_force);
}

Vec12 RobotModel::jointAngles(const Vec34& feet_position, FrameType frame) const {
    Vec12 q = Vec12::Zero();
    for (int leg = 0; leg < NumLeg; ++leg) {
        q.segment(leg * 3, 3) = jointAngles(feet_position.col(leg), leg, frame);
    }
    return q;
}

Vec12 RobotModel::jointVelocity(const Vec34& feet_position,
                                const Vec34& feet_velocity,
                                FrameType frame) const {
    Vec12 qd = Vec12::Zero();
    for (int leg = 0; leg < NumLeg; ++leg) {
        const Vec3 q = jointAngles(feet_position.col(leg), leg, frame);
        qd.segment(leg * 3, 3) = jointVelocity(q, feet_velocity.col(leg), leg);
    }
    return qd;
}

Vec12 RobotModel::jointTorques(const Vec12& q, const Vec34& foot_forces) const {
    Vec12 tau = Vec12::Zero();
    for (int leg = 0; leg < NumLeg; ++leg) {
        tau.segment(leg * 3, 3) = jointTorques(q.segment(leg * 3, 3), foot_forces.col(leg), leg);
    }
    return tau;
}

Vec3 RobotModel::footPosition(const LowlevelState& state, int leg_id, FrameType frame) const {
    return footPosition(state.getQ().col(leg_id), leg_id, frame);
}

Vec3 RobotModel::footVelocity(const LowlevelState& state, int leg_id) const {
    return footVelocity(state.getQ().col(leg_id), state.getQd().col(leg_id), leg_id);
}

Vec34 RobotModel::feetPositions(const LowlevelState& state, FrameType frame) const {
    Vec34 feet = Vec34::Zero();
    for (int leg = 0; leg < NumLeg; ++leg) {
        feet.col(leg) = footPosition(state, leg, frame == FrameType::GLOBAL ? FrameType::BODY : frame);
    }
    if (frame == FrameType::GLOBAL) {
        feet = state.getRotMat() * feet;
    }
    return feet;
}

Vec34 RobotModel::feetVelocities(const LowlevelState& state, FrameType frame) const {
    Vec34 feet_velocities = Vec34::Zero();
    for (int leg = 0; leg < NumLeg; ++leg) {
        feet_velocities.col(leg) = footVelocity(state, leg);
    }

    if (frame == FrameType::GLOBAL) {
        // 全局速度 = 机体角速度引起的附加项 + 机体系足端速度，再整体转到全局系。
        const Vec34 feet_positions_body = feetPositions(state, FrameType::BODY);
        feet_velocities += skew(state.getGyro()) * feet_positions_body;
        feet_velocities = state.getRotMat() * feet_velocities;
    }
    return feet_velocities;
}

Vec34 RobotModel::normalStandFeetInBody() const {
    return parameters_.normalStandFeetInBody();
}

}  // namespace qr_guide
