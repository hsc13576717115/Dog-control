#include "qr_planning/FiniteTemplate.hpp"
#include <gtest/gtest.h>
using qr_planning::ParseFiniteTemplate;
static YAML::Node Template() {
  return YAML::Load("frame: odom\ntarget_mode: initial_foot_offset\nsteps:\n  "
                    "- {foot: 0, surface_id: 0, offset: [0.025, 0, 0]}\n");
}
TEST(FiniteTemplate, UnitsFootOrderAndLimits) {
  const auto valid = ParseFiniteTemplate(Template());
  ASSERT_EQ(valid.size(), 1u);
  EXPECT_EQ(valid[0].foot, 0);
  EXPECT_DOUBLE_EQ(valid[0].offset[0], .025);
  auto n = Template();
  n["steps"][0]["foot"] = 4;
  EXPECT_ANY_THROW(ParseFiniteTemplate(n));
  n = Template();
  n["steps"][0]["offset"][0] = .101;
  EXPECT_ANY_THROW(ParseFiniteTemplate(n));
  n = Template();
  n["steps"][0]["offset"][0] = ".nan";
  EXPECT_ANY_THROW(ParseFiniteTemplate(n));
  n = Template();
  n["frame"] = "map";
  EXPECT_ANY_THROW(ParseFiniteTemplate(n));
}
TEST(FiniteTemplate, EmptyUnboundedAndMisspelledInputsAreRejected) {
  auto n = Template();
  n["steps"] = YAML::Node(YAML::NodeType::Sequence);
  EXPECT_ANY_THROW(ParseFiniteTemplate(n));
  const auto step = Template()["steps"][0];
  for (int i = 0; i < 17; ++i)
    n["steps"].push_back(step);
  EXPECT_ANY_THROW(ParseFiniteTemplate(n));
  n = Template();
  n["steps"][0]["typo"] = 1;
  EXPECT_ANY_THROW(ParseFiniteTemplate(n));
}

TEST(FiniteTemplate, ExtendedTravelRequiresExplicitBoundedEnvelope) {
  auto n = Template();
  n["steps"][0]["offset"][0] = .56;
  EXPECT_ANY_THROW(ParseFiniteTemplate(n));
  n["max_initial_offset_m"] = .6;
  ASSERT_NO_THROW(ParseFiniteTemplate(n));
  n["max_initial_offset_m"] = 1.01;
  EXPECT_ANY_THROW(ParseFiniteTemplate(n));
  n["max_initial_offset_m"] = ".nan";
  EXPECT_ANY_THROW(ParseFiniteTemplate(n));
}
