#include "custom_dog_control/control/SampleFreshness.hpp"
#include <gtest/gtest.h>
#include <limits>
using custom_dog_control::SampleFreshness;
TEST(SampleFreshness, RepeatedPublicationDoesNotExtendLifetime) {
  SampleFreshness guard;
  EXPECT_TRUE(guard.Accept(1., 1., true, .1));
  EXPECT_TRUE(guard.Accept(1.05, 1., true, .1));
  EXPECT_FALSE(guard.Accept(1.101, 1., true, .1));
}
TEST(SampleFreshness, ReorderedAndFutureSamplesAreRejected) {
  SampleFreshness guard;
  EXPECT_TRUE(guard.Accept(1., .99, true, .1));
  EXPECT_FALSE(guard.Accept(1.01, .98, true, .1));
  EXPECT_FALSE(guard.Accept(1.01, 1.02, true, .1));
  EXPECT_TRUE(guard.Accept(1.02, 1.02, true, .1));
  EXPECT_FALSE(guard.Accept(1., 1.02, true, .1));
}
TEST(SampleFreshness, MissingTimestampIsNotAcquisitionEvidence) {
  SampleFreshness guard;
  EXPECT_FALSE(guard.Accept(1., 1., false, .1));
  EXPECT_FALSE(
      guard.Accept(1., std::numeric_limits<double>::quiet_NaN(), true, .1));
}
