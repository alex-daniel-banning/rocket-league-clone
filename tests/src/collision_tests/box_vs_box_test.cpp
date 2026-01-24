#include <Print.hpp>
#include <engine/physics/Collisions.hpp>
#include <gtest/gtest.h>

namespace
{
// Struct for sphere vs box detection test case
struct BoxBoxDetectionCase
{
    glm::vec3 box_a_position;
    glm::vec3 box_a_rotation_euler;
    glm::vec3 box_b_position;
    glm::vec3 box_b_rotation_euler;
    bool expect_collision;
    std::string label;
};

void PrintTo(const BoxBoxDetectionCase &c, std::ostream *os) { *os << c.label; }
} // namespace

// Parameterized test for sphere vs box collision detection
class BoxBoxDetection : public ::testing::TestWithParam<BoxBoxDetectionCase>
{
};

TEST_P(BoxBoxDetection, _)
{
    const auto &c = GetParam();
    glm::quat q_a = glm::quat(glm::vec3(glm::radians(c.box_a_rotation_euler.x),
                                        glm::radians(c.box_a_rotation_euler.y),
                                        glm::radians(c.box_a_rotation_euler.z)));
    engine::physics::Box box_a(glm::vec3(1.0f), c.box_a_position, glm::vec3(0.0f), 1.0f, q_a);
    glm::quat q_b = glm::quat(glm::vec3(glm::radians(c.box_b_rotation_euler.x),
                                        glm::radians(c.box_b_rotation_euler.y),
                                        glm::radians(c.box_b_rotation_euler.z)));
    engine::physics::Box box_b(glm::vec3(1.0f), c.box_b_position, glm::vec3(0.0f), 1.0f, q_b);

    engine::physics::Contact contact;
    bool collided = engine::physics::Collisions::computeContact(box_a, box_b, contact);

    std::string msg = "Failure for Box v. Box collision detection. TEST CASE -> " + c.label;
    EXPECT_EQ(c.expect_collision, collided) << msg;
}

INSTANTIATE_TEST_SUITE_P(
    , BoxBoxDetection, // Empty prefix
    ::testing::Values(BoxBoxDetectionCase{.box_a_position       = glm::vec3(0.0f),
                                          .box_a_rotation_euler = glm::vec3(0.0f),
                                          .box_b_position       = glm::vec3(10.01f, 0.0f, 0.0f),
                                          .box_b_rotation_euler = glm::vec3(0.0f),
                                          .expect_collision     = false,
                                          .label = "FacesJustTouching_DoesNotCollide"}),
    [](const testing::TestParamInfo<BoxBoxDetectionCase> &info) { return info.param.label; });
