#include <engine/physics/Collisions.hpp>
#include <gtest/gtest.h>
#include <utility>
#include "test_util.hpp"

namespace
{
// Struct for sphere vs box detection test case
struct SphereBoxDetectionCase
{
    glm::vec3 sphere_pos;
    float sphere_radius;
    glm::quat box_rotation;
    bool expect_collision;
    std::string label;
};

void PrintTo(const SphereBoxDetectionCase &c, std::ostream *os) { *os << c.label; }
} // namespace

// Parameterized test for sphere vs box collision detection
class SphereBoxDetection : public ::testing::TestWithParam<SphereBoxDetectionCase>
{
};

TEST_P(SphereBoxDetection, _)
{
    const auto &c = GetParam();

    engine::physics::Sphere sphere(c.sphere_radius, 1.0f, c.sphere_pos, glm::vec3(0.0f));
    engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 1.0f,
                             c.box_rotation);

    engine::physics::Contact contact;
    bool collided = engine::physics::Collisions::computeContact(box, sphere, contact);

    std::string msg = "Failure for Sphere v. Box collision detection. TEST CASE -> " + c.label;
    EXPECT_EQ(c.expect_collision, collided) << msg;
}

INSTANTIATE_TEST_SUITE_P(
    , SphereBoxDetection, // Empty prefix
    ::testing::Values(
        SphereBoxDetectionCase{.sphere_pos       = glm::vec3(1.5f, 0.0f, 0.0f),
                               .sphere_radius    = 1.0f,
                               .box_rotation     = glm::quat(),
                               .expect_collision = false,
                               .label            = "JustTouchingFace_DoesNotCollide"},
        SphereBoxDetectionCase{.sphere_pos       = glm::vec3(1.499f, 0.0f, 0.0f),
                               .sphere_radius    = 1.0f,
                               .box_rotation     = glm::quat(),
                               .expect_collision = true,
                               .label            = "BarelyOverlappingFace_DoesCollide"},
        SphereBoxDetectionCase{
            .sphere_pos    = glm::vec3(0.5f + 1.0f - 0.01f, 0.5f + std::sqrt(3.0f) - 0.01f, 0.0f),
            .sphere_radius = 2.0f,
            .box_rotation  = glm::quat(),
            .expect_collision = true,
            .label            = "OffsetEdgeCollision_DoesCollide"},
        SphereBoxDetectionCase{.sphere_pos       = glm::vec3(0.5f + 1.0f - 0.01f,
                                                             0.5f + std::sqrt(3.0f) - 0.01f,
                                                             0.5f + 1.0f - 0.01f),
                               .sphere_radius    = std::sqrt(5.0f),
                               .box_rotation     = glm::quat(),
                               .expect_collision = true,
                               .label            = "SymmetricalCornerCollision_DoesCollide"},
        SphereBoxDetectionCase{.sphere_pos       = glm::vec3(2.0f, 2.0f, 2.0f),
                               .sphere_radius    = 1.0f,
                               .box_rotation     = glm::quat(),
                               .expect_collision = false,
                               .label            = "NoOverlap_DoesNotCollide"},
        SphereBoxDetectionCase{
            .sphere_pos    = glm::vec3((std::sqrt(2 * (0.5f * 0.5f))) + 1.0f - 0.01f, 0.0f, 0.0f),
            .sphere_radius = 1.0f,
            .box_rotation  = glm::quat(glm::angleAxis(glm::radians(45.0f), glm::vec3(0, 1, 0))),
            .expect_collision = true,
            .label            = "TouchingDueToRotation_DoesCollide"}),
    [](const testing::TestParamInfo<SphereBoxDetectionCase> &info) { return info.param.label; });

// Individual tests for specific behaviors
TEST(SphereBoxRotation, OffCenterImpact_ProducesAngularVelocity)
{
    // Sphere hits right side of box (+X face, above center)
    engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.499f, 0.3f, 0.0f),
                                   glm::vec3(-2.0f, 0.0f, 0.0f));
    engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f, glm::quat());

    engine::physics::Contact contact;
    ASSERT_TRUE(engine::physics::Collisions::computeContact(box, sphere, contact));

    engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

    // Impact above center on +X face should cause rotation about +Z axis
    EXPECT_GT(box.angular_velocity.z, 0.0f)
        << "Box should rotate counter-clockwise (positive Z angular velocity)";
}

TEST(SphereBoxRotation, HeavierBox_RotatesLess)
{
    // Setup: identical collision scenario, different box masses
    auto runCollision = [](float box_mass) -> std::pair<bool, float>
    {
        engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.499f, 0.3f, 0.0f),
                                       glm::vec3(-2.0f, 0.0f, 0.0f));
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), box_mass,
                                 glm::quat());

        engine::physics::Contact contact;
        bool collided = engine::physics::Collisions::computeContact(box, sphere, contact);
        if (collided)
        {
            engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);
        }
        return {collided, glm::length(box.angular_velocity)};
    };

    auto [light_collided, light_box_rotation] = runCollision(1.0f);
    auto [heavy_collided, heavy_box_rotation] = runCollision(50.0f);

    ASSERT_TRUE(light_collided) << "Light box collision not detected";
    ASSERT_TRUE(heavy_collided) << "Heavy box collision not detected";

    EXPECT_GT(light_box_rotation, heavy_box_rotation)
        << "Light box should rotate more than heavy box for same impact";
}

TEST(SphereBoxRotation, HeavierSphere_ImpartsMoreRotation)
{
    auto runCollision = [](float sphere_mass) -> std::pair<bool, float>
    {
        engine::physics::Sphere sphere(1.0f, sphere_mass, glm::vec3(1.499f, 0.3f, 0.0f),
                                       glm::vec3(-2.0f, 0.0f, 0.0f));
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f,
                                 glm::quat());

        engine::physics::Contact contact;
        bool collided = engine::physics::Collisions::computeContact(box, sphere, contact);

        if (collided)
        {
            engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);
        }
        return {collided, glm::length(box.angular_velocity)};
    };

    auto [light_collided, light_sphere_impact] = runCollision(1.0f);
    auto [heavy_collided, heavy_sphere_impact] = runCollision(10.0f);

    ASSERT_TRUE(light_collided) << "Light sphere collision not detected";
    ASSERT_TRUE(heavy_collided) << "Heavy sphere collision not detected";

    EXPECT_GT(heavy_sphere_impact, light_sphere_impact)
        << "Heavier sphere should impart more rotation for same velocity";
}

TEST(SphereBoxRotation, LargerLeverArm_CausesMoreRotation)
{
    auto runCollision = [](float y_offset) -> std::pair<bool, float>
    {
        engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.499f, y_offset, 0.0f),
                                       glm::vec3(-2.0f, 0.0f, 0.0f));
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f,
                                 glm::quat());

        engine::physics::Contact contact;
        bool collides = engine::physics::Collisions::computeContact(box, sphere, contact);
        if (collides)
        {
            engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);
        }

        return {collides, glm::length(box.angular_velocity)};
    };

    auto [near_collided, near_center]    = runCollision(0.1f);
    auto [far_collided, far_from_center] = runCollision(0.4f);

    ASSERT_TRUE(near_collided) << "Near-center sphere collision not detected";
    ASSERT_TRUE(far_collided) << "Far-from-center sphere collision not detected";

    // Lever arm is 4x larger (0.4 vs 0.1), so rotation should be ~4x more
    // Being conservative, we expect at least 3x more rotation
    EXPECT_GT(far_from_center, 3.0f * near_center)
        << "Impact at 0.4 should cause significantly more rotation than at 0.1 "
        << "(expected ~4x, got " << far_from_center / near_center << "x)";
}

TEST(SphereBoxRotation, TangentialImpact_CausesMoreRotation)
{
    auto runCollision = [](const glm::vec3 &velocity) -> std::pair<bool, float>
    {
        engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.499f, 0.3f, 0.0f), velocity);
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f,
                                 glm::quat());
        engine::physics::Contact contact;
        bool collides = engine::physics::Collisions::computeContact(box, sphere, contact);
        if (collides)
        {
            engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);
        }
        return {collides, glm::length(box.angular_velocity)};
    };

    // Tangential: sphere moving with perpendicular component (same speed, different angle)
    auto [tangential_collided, tangential_rotation] = runCollision(glm::vec3(-2.0f, 0.0f, 0.0f));

    // Direct head-on: sphere moving straight toward contact point
    auto [direct_collided, direct_rotation] = runCollision(glm::vec3(-1.4f, -1.4f, 0.0f));

    ASSERT_TRUE(tangential_collided) << "Tangential impact collision not detected";
    ASSERT_TRUE(direct_collided) << "Direct impact collision not detected";

    // Tangential impact should cause at least 50% more rotation
    EXPECT_GT(tangential_rotation, 1.2f * direct_rotation)
        << "More tangential impact should cause more rotation " << "(expected >1.2x, got "
        << tangential_rotation / direct_rotation << "x)";
}

TEST(SphereBoxRotation, HeadOnImpact_ProducesMinimalRotation)
{
    // Perfect center-to-center collision
    engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.499f, 0.0f, 0.0f),
                                   glm::vec3(-2.0f, 0.0f, 0.0f));
    engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f, glm::quat());

    engine::physics::Contact contact;
    ASSERT_TRUE(engine::physics::Collisions::computeContact(box, sphere, contact));

    engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

    EXPECT_NEAR(glm::length(box.angular_velocity), 0.0f, 1e-4f)
        << "Head-on collision should produce negligible rotation";
}

TEST(SphereBoxResolution, ZeroRelativeVelocity_OnlyAdjustsPosition)
{
    // Objects touching with matched velocities - should have no collision response
    engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.4f, 0.0f, 0.0f),
                                   glm::vec3(-1.0f, 0.0f, 0.0f));
    engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), 5.0f,
                             glm::quat());

    glm::vec3 sphere_vel_before = sphere.velocity;
    glm::vec3 box_vel_before    = box.velocity;
    glm::vec3 sphere_pos_before = sphere.position;
    glm::vec3 box_pos_before    = box.position;

    engine::physics::Contact contact;
    ASSERT_TRUE(engine::physics::Collisions::computeContact(box, sphere, contact));
    engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

    // Velocities should remain unchanged (or change minimally due to penetration resolution)
    test_util::ExpectVec3Near(sphere_vel_before, sphere.velocity, "Sphere velocity", 1e-6f);
    test_util::ExpectVec3Near(box_vel_before, box.velocity, "Box velocity", 1e-6f);

    // Positions SHOULD change to correct penetration
    EXPECT_FALSE(test_util::Vec3Near(sphere_pos_before, sphere.position, 1e-6f))
        << "Sphere position should change to resolve penetration";
    EXPECT_FALSE(test_util::Vec3Near(box_pos_before, box.position, 1e-6f))
        << "Box position should change to resolve penetration";

    // Verify penetration is resolved
    engine::physics::Contact contact_after;
    bool still_colliding = engine::physics::Collisions::computeContact(box, sphere, contact_after);
    EXPECT_FALSE(still_colliding) << "Objects should no longer be penetrating after resolution";
}
