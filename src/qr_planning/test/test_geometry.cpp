#include "qr_planning/Geometry.hpp"
#include <gtest/gtest.h>
TEST(Region, RejectUnknownEdgeAndClockwise) {
  const std::vector<Eigen::Vector2d> p = {
      {-.1, -.1}, {.1, -.1}, {.1, .1}, {-.1, .1}};
  EXPECT_TRUE(qr_planning::Inside(p, {0, 0}, .046));
  EXPECT_FALSE(qr_planning::Inside(p, {.06, 0}, .046));
  EXPECT_FALSE(qr_planning::Inside({}, {0, 0}, 0));
  auto reversed = p;
  std::reverse(reversed.begin(), reversed.end());
  EXPECT_FALSE(qr_planning::Inside(reversed, {0, 0}, 0));
}
TEST(Trajectory, ZeroEndpointVelocityAcceleration) {
  for (double t : {0., 2.}) {
    auto b = qr_planning::Quintic(t, 2);
    EXPECT_DOUBLE_EQ(b.v, 0);
    EXPECT_DOUBLE_EQ(b.a, 0);
  }
  EXPECT_NEAR(qr_planning::Quintic(1, 2).p, .5, 1e-12);
}
