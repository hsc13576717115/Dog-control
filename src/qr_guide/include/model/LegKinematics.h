#ifndef QR_GUIDE_MODEL_LEGKINEMATICS_H
#define QR_GUIDE_MODEL_LEGKINEMATICS_H

#include "common/enumClass.h"
#include "common/mathTypes.h"

namespace qr_guide {

// 单条腿的运动学模型。
// 这里严格对齐用户原始 State_SwingTest 中的 fkCheckAxis / ikCheckAxis 关节定义，
// 只是把 L0/L1/L2、腿序和髋安装点改成参数驱动。
// 坐标约定统一为：x 向前, y 向机器人右侧, z 向上。
class LegKinematics {
public:
    LegKinematics(int leg_id, double l0, double l1, double l2, const Vec3& hip_mount_in_body);

    // 前向运动学：关节角 -> 足端位置。
    Vec3 footInHip(const Vec3& q) const;
    Vec3 footInBody(const Vec3& q) const;
    // 雅可比相关接口。
    Vec3 footVelocityInHip(const Vec3& q, const Vec3& qd) const;
    // 逆运动学：足端位置 -> 关节角。
    Vec3 jointAnglesFromFoot(const Vec3& foot, FrameType frame) const;
    Vec3 jointVelocityFromFootVelocity(const Vec3& q, const Vec3& foot_velocity) const;
    Vec3 jointVelocityFromFootVelocity(const Vec3& foot, const Vec3& foot_velocity, FrameType frame) const;
    Vec3 jointTorquesFromFootForce(const Vec3& q, const Vec3& foot_force) const;
    Mat3 jacobian(const Vec3& q) const;
    const Vec3& hipMountInBody() const { return hip_mount_in_body_; }

private:
    // 将用户关节角映射到几何求解内部使用的镜像坐标。
    Vec3 toMirroredAngles(const Vec3& q_user) const;
    // 将镜像坐标下的角度映射回用户关节定义。
    Vec3 fromMirroredAngles(const Vec3& q_mirrored) const;
    bool isFrontLeg() const;
    bool isLeftLeg() const;

    int leg_id_;
    double l0_;
    double l1_;
    double l2_;
    Vec3 hip_mount_in_body_;
};

}  // namespace qr_guide

#endif  // QR_GUIDE_MODEL_LEGKINEMATICS_H
