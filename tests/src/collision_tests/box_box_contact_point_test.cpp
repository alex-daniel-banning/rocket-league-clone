#include <engine/physics/collisions.hpp>
#include <gtest/gtest.h>

#include "engine/physics/Contact.hpp"
#include "engine/physics/box.hpp"

// TODO, add anonymous namespace?
struct BoxBoxContactPointCase {
  glm::vec3 box_a_position;
  glm::quat box_a_rotation;
  glm::vec3 box_b_position;
  glm::quat box_b_rotation;
  size_t expected_point_count;
  std::vector<glm::vec3> expected_points;
  std::string label;
};

void PrintTo(const BoxBoxContactPointCase& c, std::ostream* os) {
  *os << c.label;
}

class BoxBoxContactPoints
    : public ::testing::TestWithParam<BoxBoxContactPointCase> {};

bool ContainsPoint(const std::vector<glm::vec3>& points, glm::vec3 target,
                   float epsilon = 0.01f) {
  for (const auto& p : points) {
    if (std::abs(p.x - target.x) < epsilon &&
        std::abs(p.y - target.y) < epsilon &&
        std::abs(p.z - target.z) < epsilon)
      return true;
  }
  return false;
}

TEST_P(BoxBoxContactPoints, _) {
  const auto& c = GetParam();
  engine::physics::Box box_a(glm::vec3(1.0f), c.box_a_position, glm::vec3(0.0f),
                             1.0f, c.box_a_rotation);
  engine::physics::Box box_b(glm::vec3(1.0f), c.box_b_position, glm::vec3(0.0f),
                             1.0f, c.box_b_rotation);

  engine::physics::Contact contact;
  ASSERT_TRUE(
      engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  ASSERT_EQ(contact.points.size(), c.expected_point_count) << c.label;

  for (const auto& expected : c.expected_points) {
    EXPECT_TRUE(ContainsPoint(contact.points, expected))
        << c.label << ": missing expected point (" << expected.x << ", "
        << expected.y << ", " << expected.z << ")";
  }
}

INSTANTIATE_TEST_SUITE_P(
    , BoxBoxContactPoints,
    ::testing::Values(
        // Face/Face: boxes aligned, overlapping on X axis
        // Contact face at x=0.5, full face overlap
        BoxBoxContactPointCase{.box_a_position = glm::vec3(0.0f),
                               .box_a_rotation = glm::quat(),
                               .box_b_position = glm::vec3(0.9f, 0.0f, 0.0f),
                               .box_b_rotation = glm::quat(),
                               .expected_point_count = 4,
                               .expected_points = {{0.45f, 0.5f, 0.5f},
                                                   {0.45f, 0.5f, -0.5f},
                                                   {0.45f, -0.5f, 0.5f},
                                                   {0.45f, -0.5f, -0.5f}},
                               .label = "FaceFace_Returns4Points"}),

    /*
// Face/Face: partial overlap (box_b offset in Y)
// Should clip to 4 points of smaller rectangle
BoxBoxContactPointCase{
.box_a_position = glm::vec3(0.0f),
.box_a_rotation = glm::quat(),
.box_b_position = glm::vec3(0.9f, 0.3f, 0.0f),
.box_b_rotation = glm::quat(),
.expected_point_count = 4,
.expected_points = {{0.45f, 0.5f, 0.5f},
     {0.45f, 0.5f, -0.5f},
     {0.45f, -0.2f, 0.5f},
     {0.45f, -0.2f, -0.5f}},
.label = "FaceFacePartialOverlap_Returns4ClippedPoints"}),

// Edge/Face: box_b rotated 45° around Z, edge contacts face
BoxBoxContactPointCase{
.box_a_position = glm::vec3(0.0f),
.box_a_rotation = glm::quat(),
.box_b_position = glm::vec3(0.5f + std::sqrt(0.5f) - 0.05f, 0.0f,
0.0f),
.box_b_rotation = GetRotationFromEulerAngles(glm::vec3(0.0f, 0.0f,
               45.0f)),
.expected_point_count = 2,
.expected_points = {{0.5f, 0.0f, 0.5f}, {0.5f, 0.0f, -0.5f}},
.label = "EdgeFace_Returns2Points"},

// Corner/Face: box_b diagonal-aligned, corner contacts face
BoxBoxContactPointCase{
.box_a_position = glm::vec3(0.0f),
.box_a_rotation = glm::quat(),
.box_b_position = glm::vec3(0.5f + std::sqrt(0.75f) - 0.05f, 0.0f,
0.0f),
.box_b_rotation = GetDiagonalAlignedOrientation({1, 0, 0}),
.expected_point_count = 1,
.expected_points = {{0.5f, 0.0f, 0.0f}},
.label = "CornerFace_Returns1Point"},

// Edge/Edge: both boxes rotated, edges meet
BoxBoxContactPointCase{
.box_a_position = glm::vec3(0.0f),
.box_a_rotation = GetRotationFromEulerAngles(glm::vec3(0.0f, 45.0f,
               0.0f)),
.box_b_position = glm::vec3(std::sqrt(0.5f) * 2.0f - 0.05f, 0.0f,
0.0f),
.box_b_rotation = GetRotationFromEulerAngles(glm::vec3(0.0f, 45.0f,
               0.0f)),
.expected_point_count = 2,
.expected_points = {{std::sqrt(0.5f) - 0.025f, 0.5f, 0.0f},
{std::sqrt(0.5f) - 0.025f, -0.5f, 0.0f}},
.label = "EdgeEdgeParallel_Returns2Points"}),*/
    [](const testing::TestParamInfo<BoxBoxContactPointCase>& info) {
      return info.param.label;
    });
