#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include <memory>

#include "config/RobotConfig.h"
#include "message/LowlevelState.h"
#include "model/RobotModel.h"

// 兼容旧接口的机器人壳层。
// 旧状态机和估计器仍然通过 QuadrupedRobot 调用，但内部已经切到新的 RobotModel。
class QuadrupedRobot {
public:
    explicit QuadrupedRobot(const qr_guide::RobotParameters& parameters);
    explicit QuadrupedRobot(std::shared_ptr<qr_guide::RobotModel> model);
    virtual ~QuadrupedRobot() = default;

    Vec3 getX(LowlevelState& state);
    Vec34 getVecXP(LowlevelState& state);

    Vec12 getQ(const Vec34& feetPosition, FrameType frame);
    Vec12 getQd(const Vec34& feetPosition, const Vec34& feetVelocity, FrameType frame);
    Vec12 getTau(const Vec12& q, const Vec34 feetForce);

    Vec3 getFootPosition(LowlevelState& state, int id, FrameType frame);
    Vec3 getFootVelocity(LowlevelState& state, int id);
    Vec34 getFeet2BPositions(LowlevelState& state, FrameType frame);
    Vec34 getFeet2BVelocities(LowlevelState& state, FrameType frame);
    Mat3 getJaco(LowlevelState& state, int legID);

    Vec3 forwardKinematics(const Vec3& q, int legID, FrameType frame) const;
    Vec3 inverseKinematics(const Vec3& foot, int legID, FrameType frame) const;
    const qr_guide::RobotParameters& getParameters() const;

    Vec2 getRobVelLimitX() { return getParameters().velocity_limit_x; }
    Vec2 getRobVelLimitY() { return getParameters().velocity_limit_y; }
    Vec2 getRobVelLimitYaw() { return getParameters().velocity_limit_yaw; }
    Vec34 getFeetPosIdeal() { return model_->normalStandFeetInBody(); }
    double getRobMass() { return getParameters().mass.total_mass_kg; }
    Vec3 getPcb() { return getParameters().mass.com_offset_m; }
    Mat3 getRobInertial() { return getParameters().mass.whole_robot_inertia_kg_m2_diag.asDiagonal(); }

protected:
    std::shared_ptr<qr_guide::RobotModel> model_;
};

#endif  // UNITREEROBOT_H
