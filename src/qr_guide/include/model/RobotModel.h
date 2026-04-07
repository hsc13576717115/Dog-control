#ifndef QR_GUIDE_MODEL_ROBOTMODEL_H
#define QR_GUIDE_MODEL_ROBOTMODEL_H

#include <array>
#include <memory>

#include "config/RobotConfig.h"
#include "model/LegKinematics.h"
#include "message/LowlevelState.h"

namespace qr_guide {

// 整机模型：负责把四条腿的运动学和机身参数组织成统一接口。
class RobotModel {
public:
    explicit RobotModel(const RobotParameters& parameters);

    const RobotParameters& parameters() const { return parameters_; }
    // 单腿接口。
    Vec3 footPosition(const Vec3& q, int leg_id, FrameType frame) const;
    Vec3 jointAngles(const Vec3& foot_position, int leg_id, FrameType frame) const;
    Mat3 jacobian(const Vec3& q, int leg_id) const;
    Vec3 footVelocity(const Vec3& q, const Vec3& qd, int leg_id) const;
    Vec3 jointVelocity(const Vec3& q, const Vec3& foot_velocity, int leg_id) const;
    Vec3 jointTorques(const Vec3& q, const Vec3& foot_force, int leg_id) const;

    // 四条腿批量接口。
    Vec12 jointAngles(const Vec34& feet_position, FrameType frame) const;
    Vec12 jointVelocity(const Vec34& feet_position, const Vec34& feet_velocity, FrameType frame) const;
    Vec12 jointTorques(const Vec12& q, const Vec34& foot_forces) const;

    Vec3 footPosition(const LowlevelState& state, int leg_id, FrameType frame) const;
    Vec3 footVelocity(const LowlevelState& state, int leg_id) const;
    Vec34 feetPositions(const LowlevelState& state, FrameType frame) const;
    Vec34 feetVelocities(const LowlevelState& state, FrameType frame) const;
    Vec34 normalStandFeetInBody() const;

private:
    RobotParameters parameters_;
    // legs_[i] 与腿序 FR / FL / RR / RL 一一对应。
    std::array<std::unique_ptr<LegKinematics>, NumLeg> legs_;
};

}  // namespace qr_guide

#endif  // QR_GUIDE_MODEL_ROBOTMODEL_H
