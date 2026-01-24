#include <Print.hpp>
#include <engine/physics/Collisions.hpp>
#include <gtest/gtest.h>
#include <utility>
#include "TestUtil.hpp"

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

// Struct for sphere vs box collision resolution test case
struct SphereBoxResolutionCase
{
    glm::vec3 sphere_pos;
    glm::vec3 sphere_vel;
    float sphere_radius;
    float sphere_mass;
    glm::vec3 contact_point_approx;
    glm::vec3 box_vel;
    float box_mass;
    glm::quat box_rotation;
    std::string label;
};

void PrintTo(const SphereBoxDetectionCase &c, std::ostream *os) { *os << c.label; }
void PrintTo(const SphereBoxResolutionCase &c, std::ostream *os) { *os << c.label; }

glm::vec3 expectedVelocity(const glm::vec3 &v_self, const glm::vec3 &v_other, float m_self,
                           float m_other, const glm::vec3 &normal)
{
    return v_self -
           (2.0f * m_other / (m_self + m_other)) * glm::dot(v_self - v_other, normal) * normal;
}
} // namespace

TEST(CollisionDetectionTest, BasicCollisionDetection)
{
    glm::vec3 color(0.2f, 0.15f, 0.15f);
    float planeLength = 10.0f;
    glm::vec3 planeInitialPosition(0.0f);
    glm::quat planeRotation(glm::vec3(0.0f));
    engine::physics::Plane plane(planeLength, planeLength, color, planeInitialPosition,
                                 planeRotation);
    ASSERT_EQ(glm::vec3(0.0f, 1.0f, 0.0f), plane.getNormal());

    engine::physics::Sphere sphere;
    sphere.radius   = 1.0f;
    sphere.position = glm::vec3(2.5f, sphere.radius / 2, 2.5f);
    sphere.velocity = glm::vec3(0.0f, 0.0f, 0.0f);

    EXPECT_EQ(true, engine::physics::Collisions::collides(plane, sphere));

    sphere.position = glm::vec3(2.5f, sphere.radius, 2.5f);
    EXPECT_EQ(false, engine::physics::Collisions::collides(plane, sphere));

    sphere.position = glm::vec3(2.5f, sphere.radius + 0.01f, 2.5f);
    EXPECT_EQ(false, engine::physics::Collisions::collides(plane, sphere));
}

TEST(CollisionTest, ElasticSphereVsPlaneCollision)
{
    glm::vec3 color(0.2f, 0.15f, 0.15f);
    float planeLength = 10.0f;
    glm::vec3 planeInitialPosition(0.0f);
    glm::quat planeRotation(glm::vec3(0.0f));
    engine::physics::Plane plane(planeLength, planeLength, color, planeInitialPosition,
                                 planeRotation);
    ASSERT_EQ(glm::vec3(0.0f, 1.0f, 0.0f), plane.getNormal());

    engine::physics::Sphere sphere;
    sphere.radius   = 1.0f;
    sphere.position = glm::vec3(2.5f, 0.5, 2.5f);
    sphere.velocity = glm::vec3(0.0f, -1.0f, 0.0f);

    engine::physics::Collisions::handleElasticCollision(plane, sphere);

    EXPECT_EQ(glm::vec3(0.0f, 1.0f, 0.0f), sphere.velocity);
}

TEST(CollisionTest, ElasticSphereCornerCollision)
{
    glm::vec3 color(0.2f, 0.15f, 0.15f);
    float planeLength = 10.0f;
    engine::physics::Plane plane1(planeLength, planeLength, color, glm::vec3(0.0f), glm::quat());
    TestUtil::AssertVec3Near(glm::vec3(0.0f, 1.0f, 0.0f), plane1.getNormal(),
                             "Unexpected normal for initialized plane.");
    engine::physics::Plane plane2(planeLength, planeLength, color,
                                  glm::vec3(0.0f, planeLength / 2, -planeLength / 2),
                                  glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
    TestUtil::AssertVec3Near(glm::vec3(0.0f, 0.0f, 1.0f), plane2.getNormal(),
                             "Unexpected normal for initialized plane.");

    engine::physics::Sphere sphere;
    sphere.radius   = 1.0f;
    sphere.position = glm::vec3(0.0f, 0.5f, -4.5f);
    sphere.velocity = glm::vec3(0.0f, -1.0f, -1.0f);

    engine::physics::Collisions::handleElasticCollision(plane1, sphere);
    engine::physics::Collisions::handleElasticCollision(plane2, sphere);

    TestUtil::ExpectVec3Near(glm::vec3(0.0f, 1.0f, 1.0f), sphere.velocity,
                             "Sphere did not handle two collisions in one frame as expected.");
}

TEST(CollisionReslutionTest, NoVelocityCollisionResolution)
{
    // Sphere is inside of sphere with 0 velocity
    // After collision, sphere should be telleported to "good side" (normal direction) of plane so
    // that sphere is just touching the plane.
    // This means that this collision resolution should only apply when the sphere position + radius
    // is greater than distance from plane, "not greater than or equal."
    glm::vec3 color(0.2f, 0.15f, 0.15f);
    float planeLength = 10.0f;
    glm::vec3 planeInitialPosition(0.0f);
    glm::quat planeRotation(glm::vec3(0.0f));
    engine::physics::Plane plane(planeLength, planeLength, color, planeInitialPosition,
                                 planeRotation);
    TestUtil::AssertVec3Near(glm::vec3(0.0f, 1.0f, 0.0f), plane.getNormal(),
                             "Unexpected normal for initialized plane.");

    engine::physics::Sphere sphere;
    sphere.radius   = 1.0f;
    sphere.position = glm::vec3(2.5f, (sphere.radius / 2), 2.5f);
    sphere.velocity = glm::vec3(0.0f, 0.0f, 0.0f);

    ASSERT_EQ(true, engine::physics::Collisions::collides(plane, sphere));
    engine::physics::Collisions::handleElasticCollision(plane, sphere);
    TestUtil::ExpectVec3Near(glm::vec3(2.5f, sphere.radius, 2.5f), sphere.position,
                             "Sphere shouldn't overlap with plane when it isn't moving.");
    EXPECT_EQ(glm::vec3(0.0f), sphere.velocity)
        << "Sphere shouldn't be accelerated by zero-velocity overlap.";
}

TEST(CollisionResolutionTest, ParallelVelocityCollisionResolution)
{
    // Sphere is inside of sphere with 0 velocity
    // After collision, sphere should be telleported to "good side" (normal direction) of plane so
    // that sphere is just touching the plane.
    // This means that this collision resolution should only apply when the sphere position + radius
    // is greater than distance from plane, "not greater than or equal."
    glm::vec3 color(0.2f, 0.15f, 0.15f);
    float planeLength = 10.0f;
    glm::vec3 planeInitialPosition(0.0f);
    glm::quat planeRotation(glm::vec3(0.0f));
    engine::physics::Plane plane(planeLength, planeLength, color, planeInitialPosition,
                                 planeRotation);
    TestUtil::AssertVec3Near(glm::vec3(0.0f, 1.0f, 0.0f), plane.getNormal(),
                             "Unexpected normal for initialized plane.");

    engine::physics::Sphere sphere;
    sphere.radius   = 1.0f;
    sphere.position = glm::vec3(2.5f, (sphere.radius / 2), 2.5f);
    sphere.velocity = glm::vec3(0.0f, 0.0f, -1.0f);

    ASSERT_EQ(true, engine::physics::Collisions::collides(plane, sphere));
    engine::physics::Collisions::handleElasticCollision(plane, sphere);
    TestUtil::ExpectVec3Near(glm::vec3(2.5f, sphere.radius, 2.5f), sphere.position);
    EXPECT_EQ(glm::vec3(0.0f, 0.0f, -1.0f), sphere.velocity)
        << "Overlap resultion shouldn't add velocity.";
}

// Parameterized test for sphere vs box collision detection
class SphereBoxCollisionDetectionTest : public ::testing::TestWithParam<SphereBoxDetectionCase>
{
};

TEST_P(SphereBoxCollisionDetectionTest, DetectsCorrectly)
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
    SphereVBox, SphereBoxCollisionDetectionTest,
    ::testing::Values(
        SphereBoxDetectionCase{.sphere_pos       = glm::vec3(1.5f, 0.0f, 0.0f),
                               .sphere_radius    = 1.0f,
                               .box_rotation     = glm::quat(),
                               .expect_collision = false,
                               .label            = "JustTouchingFace"},
        SphereBoxDetectionCase{.sphere_pos       = glm::vec3(1.499f, 0.0f, 0.0f),
                               .sphere_radius    = 1.0f,
                               .box_rotation     = glm::quat(),
                               .expect_collision = true,
                               .label            = "BarelyOverlappingFace"},
        SphereBoxDetectionCase{
            .sphere_pos    = glm::vec3(0.5f + 1.0f - 0.01f, 0.5f + std::sqrt(3.0f) - 0.01f, 0.0f),
            .sphere_radius = 2.0f,
            .box_rotation  = glm::quat(),
            .expect_collision = true,
            .label            = "OffsetEdgeCollision"},
        SphereBoxDetectionCase{.sphere_pos       = glm::vec3(0.5f + 1.0f - 0.01f,
                                                             0.5f + std::sqrt(3.0f) - 0.01f,
                                                             0.5f + 1.0f - 0.01f),
                               .sphere_radius    = std::sqrt(5.0f),
                               .box_rotation     = glm::quat(),
                               .expect_collision = true,
                               .label            = "SymmetricalCornerCollision"},
        SphereBoxDetectionCase{.sphere_pos       = glm::vec3(2.0f, 2.0f, 2.0f),
                               .sphere_radius    = 1.0f,
                               .box_rotation     = glm::quat(),
                               .expect_collision = false,
                               .label            = "NoOverlap"},
        SphereBoxDetectionCase{
            .sphere_pos    = glm::vec3((std::sqrt(2 * (0.5f * 0.5f))) + 1.0f - 0.01f, 0.0f, 0.0f),
            .sphere_radius = 1.0f,
            .box_rotation  = glm::quat(glm::angleAxis(glm::radians(45.0f), glm::vec3(0, 1, 0))),
            .expect_collision = true,
            .label            = "TouchingDueToRotation"}),
    [](const testing::TestParamInfo<SphereBoxDetectionCase> &info) { return info.param.label; });

/*
// Parameterized test for sphere vs box collision resolution
class SphereBoxCollisionResolutionTest : public ::testing::TestWithParam<SphereBoxResolutionCase>
{
};

TEST_P(SphereBoxCollisionResolutionTest, ResolvesCorrectly)
{
    const auto &c = GetParam();

    engine::physics::Sphere sphere(c.sphere_radius, c.sphere_mass, c.sphere_pos, c.sphere_vel);
    engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), c.box_mass,
                             c.box_rotation);

    engine::physics::Contact contact;
    ASSERT_TRUE(engine::physics::Collisions::computeContact(box, sphere, contact))
        << "Did not detect collision as expected.";
    engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

    glm::vec3 n = glm::normalize(c.sphere_pos - c.contact_point_approx);
    glm::vec3 expected_sphere_vel =
        expectedVelocity(c.sphere_vel, c.box_vel, c.sphere_mass, c.box_mass, n);
    glm::vec3 expected_box_vel =
        expectedVelocity(c.box_vel, c.sphere_vel, c.box_mass, c.sphere_mass, n);

    TestUtil::ExpectVec3Near(expected_sphere_vel, sphere.velocity, "Sphere velocity", 1e-6);
    TestUtil::ExpectVec3Near(expected_box_vel, box.velocity, "Box velocity", 1e-6);
}

INSTANTIATE_TEST_SUITE_P(
    SphereVBox, SphereBoxCollisionResolutionTest,
    ::testing::Values(
        SphereBoxResolutionCase{.sphere_pos           = glm::vec3(1.499, 0.0f, 0.0f),
                                .sphere_vel           = glm::vec3(-1.0f, 0.0f, 0.0f),
                                .sphere_radius        = 1.0f,
                                .sphere_mass          = 1.0f,
                                .contact_point_approx = glm::vec3(0.0f),
                                .box_vel              = glm::vec3(0.0f),
                                .box_mass             = 1.0f,
                                .box_rotation         = glm::quat(),
                                .label                = "FaceCollision"},
        SphereBoxResolutionCase{.sphere_pos    = glm::vec3(std::sqrt(0.5f * 0.5f + 0.5f * 0.5f),
                                                           std::sqrt(0.5f * 0.5f + 0.5f * 0.5f),
0.0f), .sphere_vel    = glm::vec3(-1.0f, 0.0f, 0.0f), .sphere_radius = 1.0f, .sphere_mass   = 1.0f,
                                .contact_point_approx = glm::vec3(0.5f, 0.5f, 0.0f),
                                .box_vel              = glm::vec3(0.0f),
                                .box_mass             = 50.0f,
                                .box_rotation         = glm::quat(),
                                .label                = "SymmetricalEdgeCollision"},
        SphereBoxResolutionCase{
            .sphere_pos = glm::vec3((0.5f + 1.0f) - 0.01f, (0.5f + std::sqrt(3.0f)) - 0.01f, 0.0f),
            .sphere_vel = glm::vec3(-2.0f, 0.0f, 0.0f),
            .sphere_radius        = 2.0f,
            .sphere_mass          = 1.0f,
            .contact_point_approx = glm::vec3(0.5f, 0.5f, 0.0f),
            .box_vel              = glm::vec3(0.0f),
            .box_mass             = 50.0f,
            .box_rotation         = glm::quat(),
            .label                = "OffsetEdgeCollision_30_60_90"},
        SphereBoxResolutionCase{.sphere_pos           = glm::vec3((0.5f + 1.0f) - 0.01f,
                                                                  (0.5f + std::sqrt(3.0f)) - 0.01f,
                                                                  (0.5f + 1.0f) - 0.01f),
                                .sphere_vel           = glm::vec3(-2.0f, 0.0f, -2.0f),
                                .sphere_radius        = std::sqrt(5.0f),
                                .sphere_mass          = 1.0f,
                                .contact_point_approx = glm::vec3(0.5f, 0.5f, 0.5f),
                                .box_vel              = glm::vec3(0.0f),
                                .box_mass             = 50.0f,
                                .box_rotation         = glm::quat(),
                                .label                = "SymmetricalCornerCollision"},
        SphereBoxResolutionCase{.sphere_pos           = glm::vec3((0.5f + 1.0f) - 0.01f,
                                                                  (0.5f + std::sqrt(3.0f)) - 0.01f,
                                                                  (0.5f + 1.0f) - 0.01f),
                                .sphere_vel           = glm::vec3(-2.0f, 0.0f, -2.0f),
                                .sphere_radius        = std::sqrt(5.0f),
                                .sphere_mass          = 1.0f,
                                .contact_point_approx = glm::vec3(0.5f),
                                .box_vel              = glm::vec3(0.0f),
                                .box_mass             = 50.0f,
                                .box_rotation         = glm::quat(),
                                .label                = "OffsetCornerCollision_30_60_90"}),
    [](const testing::TestParamInfo<SphereBoxResolutionCase> &info) { return info.param.label; });
*/

// Individual tests for specific behaviors
TEST(CollisionResolutionTest, OffCenterImpactProducesAngularVelocityInExpectedDirection)
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

TEST(CollisionResolutionTest, HeavyBoxRotatesLessThanLightBox)
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

TEST(CollisionResolutionTest, HeavySphereImpartsMoreRotationThanLightSphere)
{
    auto runCollision = [](float sphere_mass) -> std::pair<bool, float>
    {
        engine::physics::Sphere sphere(1.0f, sphere_mass, glm::vec3(1.499f, 0.3f, 0.0f),
                                       glm::vec3(-2.0f, 0.0f, 0.0f));
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f,
                                 glm::quat());

        engine::physics::Contact contact;
        bool collided = engine::physics::Collisions::computeContact(box, sphere, contact);
        engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

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

TEST(CollisionResolutionTest, FartherFromCenterCausesMoreRotation)
{
    auto runCollision = [](float y_offset) -> std::pair<bool, float>
    {
        engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.499f, y_offset, 0.0f),
                                       glm::vec3(-2.0f, 0.0f, 0.0f));
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f,
                                 glm::quat());

        engine::physics::Contact contact;
        bool collides = engine::physics::Collisions::computeContact(box, sphere, contact);
        engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

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

TEST(CollisionResolutionTest, MoreTangentialImpactCausesMoreRotation)
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

TEST(CollisionResolutionTest, HeadOnCollisionProducesMinimalRotation)
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

TEST(CollisionResolutionTest, ZeroRelativeVelocityAtContact)
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
    TestUtil::ExpectVec3Near(sphere_vel_before, sphere.velocity, "Sphere velocity", 1e-6f);
    TestUtil::ExpectVec3Near(box_vel_before, box.velocity, "Box velocity", 1e-6f);

    // Positions SHOULD change to correct penetration
    EXPECT_FALSE(TestUtil::Vec3Near(sphere_pos_before, sphere.position, 1e-6f))
        << "Sphere position should change to resolve penetration";
    EXPECT_FALSE(TestUtil::Vec3Near(box_pos_before, box.position, 1e-6f))
        << "Box position should change to resolve penetration";

    // Verify penetration is resolved
    engine::physics::Contact contact_after;
    bool still_colliding = engine::physics::Collisions::computeContact(box, sphere, contact_after);
    EXPECT_FALSE(still_colliding) << "Objects should no longer be penetrating after resolution";
}

// Base fixture with common utilities
class CollisionResolutionTest : public ::testing::Test
{
  protected:
    // Helper to compute total linear momentum
    glm::vec3 totalLinearMomentum(const engine::physics::Sphere &s, const engine::physics::Box &b)
    {
        return s.mass * s.velocity + b.mass * b.velocity;
    }

    // Helper to compute total angular momentum
    glm::vec3 totalAngularMomentum(const engine::physics::Sphere &s, const engine::physics::Box &b)
    {
        // Orbital angular momentum: L_orbital = r x p
        glm::vec3 sphere_orbital_L = glm::cross(s.position, s.mass * s.velocity);
        glm::vec3 box_orbital_L    = glm::cross(b.position, b.mass * b.velocity);

        // Spin angular momentum: L_spin = I * w
        glm::vec3 sphere_spin_L = glm::vec3(0.0f); // Not implemented yet (no friction)
        glm::vec3 box_spin_L    = b.inertia_tensor * b.angular_velocity;

        return sphere_orbital_L + box_orbital_L + sphere_spin_L + box_spin_L;
    }

    // Helper to compute total kinetic energy
    float totalKineticEnergy(const engine::physics::Sphere &s, const engine::physics::Box &b)
    {
        float linear = 0.5f * s.mass * glm::dot(s.velocity, s.velocity) +
                       0.5f * b.mass * glm::dot(b.velocity, b.velocity);
        // Only box has rotational KE: KE_rot = (1/2) * w^T * I * w
        glm::vec3 Iw     = b.inertia_tensor * b.angular_velocity;
        float rotational = 0.5f * glm::dot(b.angular_velocity, Iw);
        return linear + rotational;
    }
};

// Conservation law tests - these could be parameterized across scenarios
class ConservationLawTest : public CollisionResolutionTest,
                            public ::testing::WithParamInterface<std::string>
{
  protected:
    // Different collision scenarios
    void setupScenario(const std::string &scenario, engine::physics::Sphere &sphere,
                       engine::physics::Box &box)
    {
        if (scenario == "approaching")
        {
            sphere = engine::physics::Sphere(1.0f, 2.0f, glm::vec3(1.499f, 0.0f, 0.0f),
                                             glm::vec3(-2.0f, 0.0f, 0.0f));
            box    = engine::physics::Box(glm::vec3(1.0f), glm::vec3(0.0f),
                                          glm::vec3(1.0f, 0.0f, 0.0f), 3.0f, glm::quat());
        }
        else if (scenario == "same_direction")
        {
            sphere = engine::physics::Sphere(1.0f, 2.0f, glm::vec3(1.499f, 0.0f, 0.0f),
                                             glm::vec3(-3.0f, 0.0f, 0.0f));
            box    = engine::physics::Box(glm::vec3(1.0f), glm::vec3(0.0f),
                                          glm::vec3(-1.0f, 0.0f, 0.0f), 3.0f, glm::quat());
        }
        else if (scenario == "off_center")
        {
            sphere = engine::physics::Sphere(1.0f, 1.0f, glm::vec3(1.499f, 0.3f, 0.0f),
                                             glm::vec3(-2.0f, 0.0f, 0.0f));
            box    = engine::physics::Box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f,
                                          glm::quat());
        }
    }
};

TEST_P(ConservationLawTest, LinearMomentumIsConserved)
{
    engine::physics::Sphere sphere;
    engine::physics::Box box;
    setupScenario(GetParam(), sphere, box);

    glm::vec3 momentum_before = totalLinearMomentum(sphere, box);

    engine::physics::Contact contact;
    ASSERT_TRUE(engine::physics::Collisions::computeContact(box, sphere, contact));
    engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

    glm::vec3 momentum_after = totalLinearMomentum(sphere, box);

    TestUtil::ExpectVec3Near(momentum_before, momentum_after, "Linear momentum", 1e-5f);
}

TEST_P(ConservationLawTest, AngularMomentumIsConserved)
{
    engine::physics::Sphere sphere;
    engine::physics::Box box;
    setupScenario(GetParam(), sphere, box);

    glm::vec3 L_before = totalAngularMomentum(sphere, box);

    engine::physics::Contact contact;
    ASSERT_TRUE(engine::physics::Collisions::computeContact(box, sphere, contact));
    engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

    glm::vec3 L_after = totalAngularMomentum(sphere, box);

    TestUtil::ExpectVec3Near(L_before, L_after, "Angular momentum", 1e-5f);
}

TEST_P(ConservationLawTest, TotalKineticEnergyIsConserved)
{
    engine::physics::Sphere sphere;
    engine::physics::Box box;
    setupScenario(GetParam(), sphere, box);

    float KE_before = totalKineticEnergy(sphere, box);

    engine::physics::Contact contact;
    ASSERT_TRUE(engine::physics::Collisions::computeContact(box, sphere, contact));
    engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

    float KE_after = totalKineticEnergy(sphere, box);

    EXPECT_NEAR(KE_before, KE_after, 1e-4f)
        << "Total kinetic energy should be conserved in elastic collision";
}

INSTANTIATE_TEST_SUITE_P(DifferentScenarios, ConservationLawTest,
                         ::testing::Values("approaching", "same_direction", "off_center"));

// TODO tunneling resolution
