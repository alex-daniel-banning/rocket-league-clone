#include "engine/physics/contact.hpp"

#include <algorithm>
#include <gtest/gtest.h>

#include "engine/math.hpp"
#include "engine/physics/box.hpp"
#include "engine/physics/box_builder.hpp"
#include "engine/physics/collisions.hpp"
#include "print.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

namespace {
struct BoxBoxContactPointCase {
  glm::vec3 box_a_position;
  glm::quat box_a_rotation;
  glm::vec3 box_b_position;
  glm::quat box_b_rotation;
  size_t expected_point_count;
  std::vector<glm::vec3> expected_points;
  std::string label;
};

void PrintTo(const BoxBoxContactPointCase& c, std::ostream* os) { *os << c.label; }

std::string FormatPoints(const std::vector<glm::vec3>& points) {
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < points.size(); ++i) {
    if (i > 0) oss << ", ";
    oss << "(" << points[i].x << ", " << points[i].y << ", " << points[i].z << ")";
  }
  oss << "]";
  return oss.str();
}

std::vector<glm::vec3> ExtractPositions(const std::vector<engine::physics::ContactPoint>& points) {
  std::vector<glm::vec3> out;
  out.reserve(points.size());
  for (const auto& cp : points) out.push_back(cp.position);
  return out;
}

glm::quat GetDiagonalAlignedOrientation(glm::vec3 axis) {
  glm::vec3 diagonal = glm::normalize(glm::vec3(1, 1, 1));
  return glm::rotation(diagonal, axis);
}

void ExpectPointsEqual(std::vector<engine::physics::ContactPoint> actual,
                       std::vector<engine::physics::ContactPoint> expected, float tol = 1e-5f) {
  ASSERT_EQ(actual.size(), expected.size());
  auto cmp = [](const engine::physics::ContactPoint& a, const engine::physics::ContactPoint& b) {
    if (a.position.x != b.position.x) return a.position.x < b.position.x;
    if (a.position.y != b.position.y) return a.position.y < b.position.y;
    return a.position.z < b.position.z;
  };
  std::sort(actual.begin(), actual.end(), cmp);
  std::sort(expected.begin(), expected.end(), cmp);
  for (size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i].position.x, expected[i].position.x, tol);
    EXPECT_NEAR(actual[i].position.y, expected[i].position.y, tol);
    EXPECT_NEAR(actual[i].position.z, expected[i].position.z, tol);
  }
}
}  // namespace

class BoxBoxContactPoints : public ::testing::TestWithParam<BoxBoxContactPointCase> {};

bool ContainsPoint(const std::vector<glm::vec3>& points, glm::vec3 target, float epsilon = 0.01f) {
  for (const auto& p : points) {
    if (std::abs(p.x - target.x) < epsilon && std::abs(p.y - target.y) < epsilon && std::abs(p.z - target.z) < epsilon)
      return true;
  }
  return false;
}

TEST_P(BoxBoxContactPoints, _) {
  const auto& c = GetParam();
  engine::physics::Box box_a(glm::vec3(1.0f), c.box_a_position, glm::vec3(0.0f), 1.0f, c.box_a_rotation);
  engine::physics::Box box_b(glm::vec3(1.0f), c.box_b_position, glm::vec3(0.0f), 1.0f, c.box_b_rotation);

  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));

  ASSERT_EQ(contact.points.size(), c.expected_point_count);

  std::vector<glm::vec3> actual_positions = ExtractPositions(contact.points);

  std::vector<glm::vec3> extra;
  for (const auto& pos : actual_positions)
    if (!ContainsPoint(c.expected_points, pos)) extra.push_back(pos);

  std::vector<glm::vec3> missing;
  for (const auto& expected : c.expected_points)
    if (!ContainsPoint(actual_positions, expected)) missing.push_back(expected);

  EXPECT_TRUE(missing.empty()) << c.label << "\nMissing: " << FormatPoints(missing);
  EXPECT_TRUE(extra.empty()) << c.label << "\nExtra: " << FormatPoints(extra);
}

INSTANTIATE_TEST_SUITE_P(
    , BoxBoxContactPoints,
    ::testing::Values(
        // Face/Face: boxes aligned, overlapping on X axis
        // Contact face at x=0.5, full face overlap
        BoxBoxContactPointCase{
            .box_a_position = glm::vec3(0.0f, 0.0f, 0.0f),
            .box_a_rotation = glm::quat(),
            .box_b_position = glm::vec3(0.9f, 0.0f, 0.0f),
            .box_b_rotation = glm::quat(),
            .expected_point_count = 4,
            .expected_points = {{0.45f, 0.5f, 0.5f}, {0.45f, 0.5f, -0.5f}, {0.45f, -0.5f, 0.5f}, {0.45f, -0.5f, -0.5f}},
            .label = "FaceFace_Returns4Points"},
        BoxBoxContactPointCase{
            .box_a_position = glm::vec3(0.0f, 0.0f, 0.0f),
            .box_a_rotation = glm::quat(),
            .box_b_position = glm::vec3(0.9f, 0.0f, 0.0f),
            .box_b_rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(1, 0, 0)),
            .expected_point_count = 4,
            .expected_points = {{0.45f, 0.5f, 0.5f}, {0.45f, 0.5f, -0.5f}, {0.45f, -0.5f, 0.5f}, {0.45f, -0.5f, -0.5f}},
            .label = "FaceFace_Returns4Points_90Rotation"},
        BoxBoxContactPointCase{
            .box_a_position = glm::vec3(0.0f, 0.0f, 0.0f),
            .box_a_rotation = glm::quat(),
            .box_b_position = glm::vec3(0.9f, 0.0f, 0.0f),
            .box_b_rotation = glm::quat(0, 0, 1, 0),
            .expected_point_count = 4,
            .expected_points = {{0.45f, 0.5f, 0.5f}, {0.45f, 0.5f, -0.5f}, {0.45f, -0.5f, 0.5f}, {0.45f, -0.5f, -0.5f}},
            .label = "FaceFace_Returns4Points_180Rotation"},

        // Face/Face: partial overlap (box_b offset in Y)
        // Should clip to 4 points of smaller rectangle
        BoxBoxContactPointCase{
            .box_a_position = glm::vec3(0.0f),
            .box_a_rotation = glm::quat(),
            .box_b_position = glm::vec3(0.9f, 0.3f, 0.0f),
            .box_b_rotation = glm::quat(),
            .expected_point_count = 4,
            .expected_points = {{0.45f, 0.5f, 0.5f}, {0.45f, 0.5f, -0.5f}, {0.45f, -0.2f, 0.5f}, {0.45f, -0.2f, -0.5f}},
            .label = "FaceFacePartialOverlap_Returns4ClippedPoints"},

        // Edge/Face: box_b rotated 45° around Z, edge contacts face
        BoxBoxContactPointCase{.box_a_position = glm::vec3(0.0f),
                               .box_a_rotation = glm::quat(),
                               .box_b_position = glm::vec3(0.5f + std::sqrt(0.5f) - 0.05f, 0.0f, 0.0f),
                               .box_b_rotation = engine::Math::GetRotationFromEulerAngles(glm::vec3(0.0f, 0.0f, 45.0f)),
                               .expected_point_count = 2,
                               .expected_points = {{0.475f, 0.0f, 0.5f}, {0.475f, 0.0f, -0.5f}},
                               .label = "EdgeFace_Returns2Points"},

        // Corner/Face: box_b diagonal-aligned, corner contacts face
        BoxBoxContactPointCase{.box_a_position = glm::vec3(0.0f),
                               .box_a_rotation = glm::quat(),
                               .box_b_position = glm::vec3(0.5f + std::sqrt(0.75f) - 0.05f, 0.0f, 0.0f),
                               .box_b_rotation = GetDiagonalAlignedOrientation({1, 0, 0}),
                               .expected_point_count = 1,
                               .expected_points = {{0.475f, 0.0f, 0.0f}},
                               .label = "CornerFace_Returns1Point"}),

    [](const testing::TestParamInfo<BoxBoxContactPointCase>& info) { return info.param.label; });

TEST(BoxBoxContact, EdgeFaceClippingShouldBeIndependentOfArgumentOrder) {
  auto box_a_1 = engine::physics::BoxBuilder().Build();
  auto box_b_1 = engine::physics::BoxBuilder()
                     .Position(1.207f, 0.0f, 0.0f)
                     .Rotation(glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f)))
                     .Build();
  engine::physics::Contact contact_1;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a_1, box_b_1, contact_1));

  auto box_a_2 = engine::physics::BoxBuilder().Build();
  auto box_b_2 = engine::physics::BoxBuilder()
                     .Position(1.207f, 0.0f, 0.0f)
                     .Rotation(glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f)))
                     .Build();
  engine::physics::Contact contact_2;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_b_2, box_a_2, contact_2));

  ExpectPointsEqual(contact_1.points, contact_2.points, 1e-5f);
}

// --- Per-point penetration depth tests ---

TEST(BoxBoxContact, PerPointDepthsArePositive) {
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder().Position(0.9f, 0.0f, 0.0f).Build();
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  ASSERT_FALSE(contact.points.empty());
  for (const auto& cp : contact.points) EXPECT_GT(cp.penetration, 0.0f);
}

TEST(BoxBoxContact, FlatFaceFacePerPointDepthsAreUniform) {
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder().Position(0.9f, 0.0f, 0.0f).Build();
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  ASSERT_FALSE(contact.points.empty());
  for (const auto& cp : contact.points) EXPECT_NEAR(cp.penetration, contact.penetration, 1e-5f);
}

TEST(BoxBoxContact, TiltedFaceFacePerPointDepthsVary) {
  // box_b tilted 5° around Z: upper corners penetrate deeper than lower corners
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
                   .Position(0.9f, 0.0f, 0.0f)
                   .Rotation(glm::angleAxis(glm::radians(5.0f), glm::vec3(0.0f, 0.0f, 1.0f)))
                   .Build();
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  ASSERT_GE(contact.points.size(), 2u);
  float min_d = contact.points[0].penetration;
  float max_d = contact.points[0].penetration;
  for (const auto& cp : contact.points) {
    min_d = std::min(min_d, cp.penetration);
    max_d = std::max(max_d, cp.penetration);
  }
  EXPECT_GT(max_d - min_d, 1e-4f);
}

// --- Manifold reduction test ---

// box_b rotated 45° around X produces a diamond-shaped incident face whose
// 4 corners all fall outside the reference face. Sutherland-Hodgman returns
// 8 intersection points; the manifold should be reduced to at most 4.
TEST(BoxBoxContact, FaceFaceRotated45DegReducesToFourPoints) {
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
                   .Position(0.9f, 0.0f, 0.0f)
                   .Rotation(glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f)))
                   .Build();
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  EXPECT_LE(contact.points.size(), 4u);
}

TEST(BoxBoxContact, CornerFaceClippingShouldBeIndependentOfArgumentOrder) {
  glm::vec3 position(0.5f + std::sqrt(0.75f) - 0.05f, 0.0f, 0.0f);
  glm::quat rotation = GetDiagonalAlignedOrientation({1, 0, 0});

  auto box_a_1 = engine::physics::BoxBuilder().Build();
  auto box_b_1 = engine::physics::BoxBuilder().Position(position).Rotation(rotation).Build();
  engine::physics::Contact contact_1;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a_1, box_b_1, contact_1));

  auto box_a_2 = engine::physics::BoxBuilder().Build();
  auto box_b_2 = engine::physics::BoxBuilder().Position(position).Rotation(rotation).Build();
  engine::physics::Contact contact_2;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_b_2, box_a_2, contact_2));

  ExpectPointsEqual(contact_1.points, contact_2.points, 1e-5f);
}
