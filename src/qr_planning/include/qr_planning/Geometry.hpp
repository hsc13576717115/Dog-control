#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace qr_planning {
// Convex CCW polygons only. Unknown or empty regions are never walkable.
inline bool Inside(const std::vector<Eigen::Vector2d> &polygon,
                   const Eigen::Vector2d &point, double margin) {
  if (polygon.size() < 3 || !point.allFinite() || !std::isfinite(margin) ||
      margin < 0)
    return false;
  double area = 0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto &a = polygon[i];
    const auto &b = polygon[(i + 1) % polygon.size()];
    if (!a.allFinite() || !b.allFinite())
      return false;
    const Eigen::Vector2d e = b - a;
    if (e.norm() < 1e-8 ||
        e.x() * (point.y() - a.y()) - e.y() * (point.x() - a.x()) <
            margin * e.norm() - 1e-9)
      return false;
    area += a.x() * b.y() - a.y() * b.x();
  }
  return area > 1e-8;
}
struct Blend {
  double p, v, a;
};
inline Blend Quintic(double elapsed, double duration) {
  const double s = std::clamp(elapsed / duration, 0.0, 1.0);
  return {s * s * s * (10 + s * (-15 + 6 * s)),
          30 * s * s * (1 - s) * (1 - s) / duration,
          60 * s * (1 - s) * (1 - 2 * s) / (duration * duration)};
}
inline Eigen::Vector3d
SupportCentroid(const std::array<Eigen::Vector3d, 4> &feet, size_t swing) {
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < 4; ++i)
    if (i != swing)
      c += feet[i] / 3.0;
  return c;
}
} // namespace qr_planning
