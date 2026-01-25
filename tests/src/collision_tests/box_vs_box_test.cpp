#include <Print.hpp>
#include <engine/physics/Collisions.hpp>
#include <gtest/gtest.h>

namespace {
// Struct for sphere vs box detection test case
struct BoxBoxDetectionCase {
  glm::vec3 box_a_position;
  glm::quat box_a_rotation;
  glm::vec3 box_b_position;
  glm::quat box_b_rotation;
  bool expect_collision;
  std::string label;
};

void PrintTo(const BoxBoxDetectionCase &c, std::ostream *os) { *os << c.label; }

glm::quat getRotationFromEulerAngles(glm::vec3 euler) {
  return glm::quat(glm::vec3(glm::radians(euler.x), glm::radians(euler.y),
                             glm::radians(euler.z)));
}

glm::quat getDiagonalAlignedOrientation(glm::vec3 axis) {
  glm::vec3 diagonal = glm::normalize(glm::vec3(1, 1, 1));
  return glm::rotation(diagonal, axis);
}

}  // namespace

// Parameterized test for sphere vs box collision detection
class BoxBoxDetection : public ::testing::TestWithParam<BoxBoxDetectionCase> {};

TEST_P(BoxBoxDetection, _) {
  const auto &c = GetParam();
  engine::physics::Box box_a(glm::vec3(1.0f), c.box_a_position, glm::vec3(0.0f),
                             1.0f, c.box_a_rotation);
  engine::physics::Box box_b(glm::vec3(1.0f), c.box_b_position, glm::vec3(0.0f),
                             1.0f, c.box_b_rotation);

  engine::physics::Contact contact;
  bool collided =
      engine::physics::Collisions::computeContact(box_a, box_b, contact);

  std::string msg =
      "Failure for Box v. Box collision detection. TEST CASE -> " + c.label;
  EXPECT_EQ(c.expect_collision, collided) << msg;
}

INSTANTIATE_TEST_SUITE_P(
    , BoxBoxDetection,  // Empty prefix
    ::testing::Values(
        BoxBoxDetectionCase{.box_a_position = glm::vec3(0.0f),
                            .box_a_rotation = glm::quat(),
                            .box_b_position = glm::vec3(1.0f, 0.0f, 0.0f),
                            .box_b_rotation = glm::quat(),
                            .expect_collision = false,
                            .label = "FacesJustTouching_DoesNotCollide"},
        BoxBoxDetectionCase{.box_a_position = glm::vec3(0.0f),
                            .box_a_rotation = glm::quat(),
                            .box_b_position = glm::vec3(0.999f, 0.0f, 0.0f),
                            .box_b_rotation = glm::quat(),
                            .expect_collision = true,
                            .label = "FacesJustOverlapping_DoesCollide"},
        BoxBoxDetectionCase{
            .box_a_position = glm::vec3(0.0f),
            .box_a_rotation = glm::quat(),
            .box_b_position = glm::vec3(0.5f + std::sqrt(0.5f) + 0.001f, 0.0f,
                                        0.0f),
            .box_b_rotation = getRotationFromEulerAngles(glm::vec3(0.0f, 45.0f,
                                                                   0.0f)),
            .expect_collision = false,
            .label = "EdgeSeparateFromFace_DoesNotCollide"},
        BoxBoxDetectionCase{
            .box_a_position = glm::vec3(0.0f),
            .box_a_rotation = glm::quat(),
            .box_b_position = glm::vec3(0.5f + std::sqrt(0.5f) - 0.001f, 0.0f,
                                        0.0f),
            .box_b_rotation = getRotationFromEulerAngles(glm::vec3(0.0f, 45.0f,
                                                                   0.0f)),
            .expect_collision = true,
            .label = "EdgeOverlappingFace_DoesCollide"},
        BoxBoxDetectionCase{
            .box_a_position = glm::vec3(0.0f),
            .box_a_rotation = glm::quat(),
            .box_b_position = glm::vec3(0.5f + std::sqrt(0.75f) + 0.001f, 0.0f,
                                        0.0f),
            .box_b_rotation = getDiagonalAlignedOrientation({1, 0, 0}),
            .expect_collision = false,
            .label = "CornerSeparateFromFace_DoesNotCollide"},
        BoxBoxDetectionCase{
            .box_a_position = glm::vec3(0.0f),
            .box_a_rotation = glm::quat(),
            .box_b_position = glm::vec3(0.5f + std::sqrt(0.75f) - 0.001f, 0.0f,
                                        0.0f),
            .box_b_rotation = getDiagonalAlignedOrientation({1, 0, 0}),
            .expect_collision = true,
            .label = "CornerOverlappingFace_DoesCollide"},
        // below
        BoxBoxDetectionCase{
            .box_a_position = glm::vec3(0.0f),
            .box_a_rotation = getRotationFromEulerAngles(glm::vec3(0.0f, 45.0f,
                                                                   0.0f)),
            .box_b_position = glm::vec3(
                std::sqrt(0.5f) + std::sqrt(0.75f) - 0.001f, 0.0f, 0.0f),
            .box_b_rotation = getDiagonalAlignedOrientation({1, 0, 0}),
            .expect_collision = true,
            .label = "CornerOverlappingEdge_DoesCollide"},
        BoxBoxDetectionCase{
            .box_a_position = glm::vec3(0.0f),
            .box_a_rotation = getRotationFromEulerAngles(glm::vec3(0.0f, 45.0f,
                                                                   0.0f)),
            .box_b_position = glm::vec3(
                std::sqrt(0.5f) + std::sqrt(0.75f) + 0.001f, 0.0f, 0.0f),
            .box_b_rotation = getDiagonalAlignedOrientation({1, 0, 0}),
            .expect_collision = false,
            .label = "CornerSeparateFromEdge_DoesNotCollide"},
        BoxBoxDetectionCase{
            .box_a_position = glm::vec3(0.0f),
            .box_a_rotation = getDiagonalAlignedOrientation({1, 0, 0}),
            .box_b_position = glm::vec3(2.0f * std::sqrt(0.75f) - 0.001f, 0.0f,
                                        0.0f),
            .box_b_rotation = getDiagonalAlignedOrientation({1, 0, 0}),
            .expect_collision = true,
            .label = "CornersOverlapping_DoesCollide"},
        BoxBoxDetectionCase{
            .box_a_position = glm::vec3(0.0f),
            .box_a_rotation = getDiagonalAlignedOrientation({1, 0, 0}),
            .box_b_position = glm::vec3(2.0f * std::sqrt(0.75f) + 0.001f, 0.0f,
                                        0.0f),
            .box_b_rotation = getDiagonalAlignedOrientation({1, 0, 0}),
            .expect_collision = false,
            .label = "CornersSeparate_DoesNotCollide"}),
    [](const testing::TestParamInfo<BoxBoxDetectionCase> &info) {
      return info.param.label;
    });

TEST_F(BoxBoxDetection, BoxesCollideDueToScale) {
  glm::vec3 smaller_scale(0.999f);
  glm::vec3 larger_scale(1.005f);
  engine::physics::Box box_a(glm::vec3(1.0f), glm::vec3(), glm::vec3(), 1.0f,
                             glm::quat());
  engine::physics::Box box_b(
      smaller_scale, glm::vec3(0.5f + std::sqrt(0.75f) + 0.001f, 0.0f, 0.0f),
      glm::vec3(0.0f), 1.0f, getDiagonalAlignedOrientation({1, 0, 0}));

  engine::physics::Contact first_contact;
  bool smaller_box_collided =
      engine::physics::Collisions::computeContact(box_a, box_b, first_contact);

  EXPECT_FALSE(smaller_box_collided)
      << "Boxes not detecting scale-caused lack of collision. "
         "(Incorrectly detected shrunken box)";

  box_b.size = larger_scale;
  engine::physics::Contact second_contact;
  bool larger_box_collided =
      engine::physics::Collisions::computeContact(box_a, box_b, second_contact);

  EXPECT_TRUE(larger_box_collided)
      << "Boxes not detecting scale-caused collision. (Did not detect enlarged "
         "box)";
}

// RESOLUTION TEST CASES
//
// For these, the contact point is ambiguous
// TODO - Faces overlapping & parallel
// TODO - Edges overlapping & parallel
// TODO - Face and edge overlapping & parallel
