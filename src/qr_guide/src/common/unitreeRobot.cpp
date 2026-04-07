#include "common/unitreeRobot.h"

#include <iostream>

QuadrupedRobot::QuadrupedRobot(const qr_guide::RobotParameters& parameters)
    : model_(std::make_shared<qr_guide::RobotModel>(parameters)) {}

QuadrupedRobot::QuadrupedRobot(std::shared_ptr<qr_guide::RobotModel> model)
    : model_(std::move(model)) {}

Vec3 QuadrupedRobot::getX(LowlevelState& state) {
    return getFootPosition(state, 0, FrameType::BODY);
}

Vec34 QuadrupedRobot::getVecXP(LowlevelState& state) {
    Vec34 vecXP;
    const Vec3 x = getX(state);
    const Vec34 qLegs = state.getQ();
    for (int i = 0; i < 4; ++i) {
        vecXP.col(i) = forwardKinematics(qLegs.col(i), i, FrameType::BODY) - x;
    }
    return vecXP;
}

Vec12 QuadrupedRobot::getQ(const Vec34& feetPosition, FrameType frame) {
    return model_->jointAngles(feetPosition, frame);
}

Vec12 QuadrupedRobot::getQd(const Vec34& feetPosition, const Vec34& feetVelocity, FrameType frame) {
    return model_->jointVelocity(feetPosition, feetVelocity, frame);
}

Vec12 QuadrupedRobot::getTau(const Vec12& q, const Vec34 feetForce) {
    return model_->jointTorques(q, feetForce);
}

Vec3 QuadrupedRobot::getFootPosition(LowlevelState& state, int id, FrameType frame) {
    return model_->footPosition(state, id, frame);
}

Vec3 QuadrupedRobot::getFootVelocity(LowlevelState& state, int id) {
    return model_->footVelocity(state, id);
}

Vec34 QuadrupedRobot::getFeet2BPositions(LowlevelState& state, FrameType frame) {
    return model_->feetPositions(state, frame);
}

Vec34 QuadrupedRobot::getFeet2BVelocities(LowlevelState& state, FrameType frame) {
    return model_->feetVelocities(state, frame);
}

Mat3 QuadrupedRobot::getJaco(LowlevelState& state, int legID) {
    return model_->jacobian(state.getQ().col(legID), legID);
}

Vec3 QuadrupedRobot::forwardKinematics(const Vec3& q, int legID, FrameType frame) const {
    // 旧接口继续保留，内部实际调用新的参数化模型。
    return model_->footPosition(q, legID, frame);
}

Vec3 QuadrupedRobot::inverseKinematics(const Vec3& foot, int legID, FrameType frame) const {
    return model_->jointAngles(foot, legID, frame);
}

const qr_guide::RobotParameters& QuadrupedRobot::getParameters() const {
    return model_->parameters();
}

A1Robot::A1Robot()
    : QuadrupedRobot(qr_guide::MakeA1Parameters()) {}

Go1Robot::Go1Robot()
    : QuadrupedRobot(qr_guide::MakeGo1Parameters()) {}
