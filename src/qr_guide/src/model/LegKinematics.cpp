#include "model/LegKinematics.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace qr_guide {

namespace {

template <typename T>
T ClampValue(T value, T min_value, T max_value) {
    return std::min(std::max(value, min_value), max_value);
}

}  // namespace

LegKinematics::LegKinematics(int leg_id,
                             double l0,
                             double l1,
                             double l2,
                             const Vec3& hip_mount_in_body)
    : leg_id_(leg_id),
      l0_(l0),
      l1_(l1),
      l2_(l2),
      hip_mount_in_body_(hip_mount_in_body) {}

Vec3 LegKinematics::footInHip(const Vec3& q_user) const {
    // 这里严格对齐原 State_SwingTest::fkCheckAxis 的关节定义和镜像规则。
    const Vec3 q = toMirroredAngles(q_user);

    const double q0 = q(0);
    const double q1 = q(1);
    const double q2 = q(2);

    const double q1_abs = q1;
    const double q2_abs = M_PI_2 - q2;
    const double knee = q1_abs + q2_abs;

    const double x2 = l1_ * std::cos(q1_abs) + l2_ * std::cos(knee);
    const double z2 = l1_ * std::sin(q1_abs) + l2_ * std::sin(knee);
    const double y_off = isLeftLeg() ? -l0_ : l0_;

    Vec3 foot;
    foot(0) = x2;
    foot(1) = y_off * std::cos(q0) - z2 * std::sin(q0);
    foot(2) = y_off * std::sin(q0) + z2 * std::cos(q0);
    return foot;
}

Vec3 LegKinematics::footInBody(const Vec3& q) const {
    return hip_mount_in_body_ + footInHip(q);
}

Vec3 LegKinematics::footVelocityInHip(const Vec3& q, const Vec3& qd) const {
    return jacobian(q) * qd;
}

Vec3 LegKinematics::jointAnglesFromFoot(const Vec3& foot, FrameType frame) const {
    // 这里严格对齐原 State_SwingTest::ikCheckAxis 的求解流程。
    Vec3 foot_in_hip = foot;
    if (frame == FrameType::BODY) {
        foot_in_hip -= hip_mount_in_body_;
    } else if (frame != FrameType::HIP) {
        std::cerr << "[LegKinematics] jointAnglesFromFoot only supports HIP or BODY frame." << std::endl;
        std::exit(-1);
    }

    const double x = foot_in_hip.x();
    const double y = foot_in_hip.y();
    const double z = foot_in_hip.z();
    const double y_off = isLeftLeg() ? -l0_ : l0_;

    double q0 = 0.0;
    const double A = y;
    const double B = z;
    const double denom = A * A + B * B;
    if (denom > 1e-6) {
        const double root = std::sqrt(std::max(0.0, denom - y_off * y_off));
        const double sin_q0 = (B * y_off + A * root) / denom;
        const double cos_q0 = (y_off - B * sin_q0) / A;
        q0 = std::atan2(sin_q0, cos_q0);
    }

    const double x2 = -x;
    const double z2 = z * std::cos(q0) - y * std::sin(q0);
    const double r = std::hypot(x2, z2);
    const double r_min = std::fabs(l1_ - l2_);
    const double r_max = l1_ + l2_;

    static Vec3 q_last(0.0, 0.3, -0.8);
    if (r < r_min || r > r_max) {
        std::cerr << "[IK] unreachable r=" << r << std::endl;
        return q_last;
    }

    const double alpha = std::atan2(-z2, -x2);
    const double beta = std::acos(ClampValue((l1_ * l1_ + r * r - l2_ * l2_) / (2.0 * l1_ * r), -1.0, 1.0));
    const double c2 = ClampValue((l1_ * l1_ + l2_ * l2_ - r * r) / (2.0 * l1_ * l2_), -1.0, 1.0);

    Vec3 q_back(q0, -(alpha + beta), -M_PI_2 + std::acos(c2));
    Vec3 q_front(q0, -(alpha - beta), -M_PI_2 - std::acos(c2));

    const auto inside_limits = [](const Vec3& q_candidate) {
        return q_candidate(1) >= -6.50 && q_candidate(1) <= 6.50 &&
               q_candidate(2) >= -2.30 && q_candidate(2) <= 2.30;
    };

    Vec3 q_mirrored = inside_limits(q_back) ? q_back : q_front;
    Vec3 q_user = fromMirroredAngles(q_mirrored);
    q_last = q_user;
    return q_user;
}

Vec3 LegKinematics::jointVelocityFromFootVelocity(const Vec3& q, const Vec3& foot_velocity) const {
    return jacobian(q).inverse() * foot_velocity;
}

Vec3 LegKinematics::jointVelocityFromFootVelocity(const Vec3& foot,
                                                  const Vec3& foot_velocity,
                                                  FrameType frame) const {
    return jointVelocityFromFootVelocity(jointAnglesFromFoot(foot, frame), foot_velocity);
}

Vec3 LegKinematics::jointTorquesFromFootForce(const Vec3& q, const Vec3& foot_force) const {
    return jacobian(q).transpose() * foot_force;
}

Mat3 LegKinematics::jacobian(const Vec3& q_user) const {
    // Jacobian 依据当前 SwingTest 约定下的 FK 解析求导。
    const Vec3 q = toMirroredAngles(q_user);

    const double q0 = q(0);
    const double q1 = q(1);
    const double q2 = q(2);
    const double y_off = isLeftLeg() ? -l0_ : l0_;

    const double q1_abs = q1;
    const double q2_abs = M_PI_2 - q2;
    const double knee = q1_abs + q2_abs;

    const double x2 = l1_ * std::cos(q1_abs) + l2_ * std::cos(knee);
    const double z2 = l1_ * std::sin(q1_abs) + l2_ * std::sin(knee);
    const double c0 = std::cos(q0);
    const double s0 = std::sin(q0);
    const double ck = std::cos(knee);
    const double sk = std::sin(knee);

    const double dx_dq1_m = -l1_ * std::sin(q1_abs) - l2_ * sk;
    const double dz2_dq1_m = l1_ * std::cos(q1_abs) + l2_ * ck;
    const double dx_dq2_m = l2_ * sk;
    const double dz2_dq2_m = -l2_ * ck;

    const double dq0_m_dq0 = isFrontLeg() ? -1.0 : 1.0;
    const double dq1_m_dq1 = isLeftLeg() ? -1.0 : 1.0;
    const double dq2_m_dq2 = isLeftLeg() ? -1.0 : 1.0;

    Mat3 jaco = Mat3::Zero();

    // q0 列
    jaco(0, 0) = 0.0;
    jaco(1, 0) = (-y_off * s0 - z2 * c0) * dq0_m_dq0;
    jaco(2, 0) = (y_off * c0 - z2 * s0) * dq0_m_dq0;

    // q1 列
    jaco(0, 1) = dx_dq1_m * dq1_m_dq1;
    jaco(1, 1) = (-dz2_dq1_m * s0) * dq1_m_dq1;
    jaco(2, 1) = (dz2_dq1_m * c0) * dq1_m_dq1;

    // q2 列
    jaco(0, 2) = dx_dq2_m * dq2_m_dq2;
    jaco(1, 2) = (-dz2_dq2_m * s0) * dq2_m_dq2;
    jaco(2, 2) = (dz2_dq2_m * c0) * dq2_m_dq2;

    return jaco;
}

Vec3 LegKinematics::toMirroredAngles(const Vec3& q_user) const {
    Vec3 q = q_user;
    if (isFrontLeg()) {
        q(0) = -q(0);
    }
    if (isLeftLeg()) {
        q(1) = -q(1);
        q(2) = -q(2);
    }
    return q;
}

Vec3 LegKinematics::fromMirroredAngles(const Vec3& q_mirrored) const {
    Vec3 q = q_mirrored;
    if (isFrontLeg()) {
        q(0) = -q(0);
    }
    if (isLeftLeg()) {
        q(1) = -q(1);
        q(2) = -q(2);
    }
    return q;
}

bool LegKinematics::isFrontLeg() const {
    return leg_id_ == 0 || leg_id_ == 1;
}

bool LegKinematics::isLeftLeg() const {
    return leg_id_ == 1 || leg_id_ == 3;
}

}  // namespace qr_guide
