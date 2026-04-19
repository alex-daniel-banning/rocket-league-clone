#include <engine/physics/collisions.hpp>
#include <gtest/gtest.h>

namespace {
// Struct for sphere vs box detection test case
struct SphereBoxDetectionCase {
  glm::vec3 sphere_pos;
  float sphere_radius;
  glm::quat box_rotation;
  bool expect_collision;
  std::string label;
};

void PrintTo(const SphereBoxDetectionCase& c, std::ostream* os) { *os << c.label; }

}  // namespace

// Parameterized test for sphere vs box collision detection
class SphereBoxDetection : public ::testing::TestWithParam<SphereBoxDetectionCase> {};

TEST_P(SphereBoxDetection, _) {
  const auto& c = GetParam();

  engine::physics::Sphere sphere(c.sphere_radius, 1.0f, c.sphere_pos, glm::vec3(0.0f));
  engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 1.0f, c.box_rotation);

  engine::physics::Contact contact;
  bool collided = engine::physics::collisions::ComputeContact(box, sphere, contact);

  std::string msg = "Failure for Sphere v. Box collision detection. TEST CASE -> " + c.label;
  EXPECT_EQ(c.expect_collision, collided) << msg;
}

INSTANTIATE_TEST_SUITE_P(
    , SphereBoxDetection,  // Empty prefix
    ::testing::Values(
        SphereBoxDetectionCase{.sphere_pos = glm::vec3(1.5f, 0.0f, 0.0f),
                               .sphere_radius = 1.0f,
                               .box_rotation = glm::quat(),
                               .expect_collision = false,
                               .label = "JustTouchingFace_DoesNotCollide"},
        SphereBoxDetectionCase{.sphere_pos = glm::vec3(1.499f, 0.0f, 0.0f),
                               .sphere_radius = 1.0f,
                               .box_rotation = glm::quat(),
                               .expect_collision = true,
                               .label = "BarelyOverlappingFace_DoesCollide"},
        SphereBoxDetectionCase{.sphere_pos = glm::vec3(0.5f + 1.0f - 0.01f, 0.5f + std::sqrt(3.0f) - 0.01f, 0.0f),
                               .sphere_radius = 2.0f,
                               .box_rotation = glm::quat(),
                               .expect_collision = true,
                               .label = "OffsetEdgeCollision_DoesCollide"},
        SphereBoxDetectionCase{.sphere_pos = glm::vec3(0.5f + 1.0f - 0.01f, 0.5f + std::sqrt(3.0f) - 0.01f,
                                                       0.5f + 1.0f - 0.01f),
                               .sphere_radius = std::sqrt(5.0f),
                               .box_rotation = glm::quat(),
                               .expect_collision = true,
                               .label = "SymmetricalCornerCollision_DoesCollide"},
        SphereBoxDetectionCase{.sphere_pos = glm::vec3(2.0f, 2.0f, 2.0f),
                               .sphere_radius = 1.0f,
                               .box_rotation = glm::quat(),
                               .expect_collision = false,
                               .label = "NoOverlap_DoesNotCollide"},
        SphereBoxDetectionCase{.sphere_pos = glm::vec3((std::sqrt(2 * (0.5f * 0.5f))) + 1.0f - 0.01f, 0.0f, 0.0f),
                               .sphere_radius = 1.0f,
                               .box_rotation = glm::quat(glm::angleAxis(glm::radians(45.0f), glm::vec3(0, 1, 0))),
                               .expect_collision = true,
                               .label = "TouchingDueToRotation_DoesCollide"}),
    [](const testing::TestParamInfo<SphereBoxDetectionCase>& info) { return info.param.label; });
