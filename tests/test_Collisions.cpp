#include <engine/physics/Collisions.hpp>
#include <gtest/gtest.h>
#include "TestUtil.hpp"

namespace
{
// Struct for sphere vs box test case
struct SphereBoxCase
{
    glm::vec3 sphere_pos;
    glm::vec3 sphere_vel;
    float sphere_radius;
    float sphere_mass;
    glm::vec3 contact_point_approx;
    float box_mass;
};

glm::vec3 expectedVelocity(const glm::vec3 &v, float m_self, float m_other, const glm::vec3 &normal)
{
    return v - (2.0f * m_other / (m_self + m_other)) * glm::dot(v, normal) * normal;
}

// Minimal helper function to run a sphere vs box collision test
void runSphereBoxCollisionTest(const SphereBoxCase &c)
{
    engine::physics::Sphere sphere(c.sphere_radius, c.sphere_mass, c.sphere_pos, c.sphere_vel);
    engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), c.box_mass);

    engine::physics::Collisions::handleElasticCollision(box, sphere);

    glm::vec3 n         = glm::normalize(c.sphere_pos - c.contact_point_approx);
    glm::vec3 vExpected = expectedVelocity(c.sphere_vel, c.sphere_mass, c.box_mass, n);

    TestUtil::ExpectVec3Near(vExpected, sphere.velocity,
                             "Sphere velocity after collision should reflect correctly along "
                             "normal, accounting for box mass.",
                             1e-6);
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
    engine::physics::Plane plane1(planeLength, planeLength, color, glm::vec3(0.0f),
                                  glm::quat(glm::vec3(0.0f)));
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

// TODO, I think if an early block fails, it won't execute the next
TEST(CollisionDetectionTest, SphereVsBoxFace)
{
    {
        glm::vec3 sphere_velocity_initial = glm::vec3(-1.0f, 0.0f, 0.0f);
        engine::physics::Sphere sphereJustTouching(1.0f, 1.0f, glm::vec3(1.5f, 0.0f, 0.0f),
                                                   sphere_velocity_initial);
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f));
        engine::physics::Collisions::handleElasticCollision(box, sphereJustTouching);
        TestUtil::ExpectVec3Near(
            sphere_velocity_initial, sphereJustTouching.velocity,
            "Sphere that just touches the surface should not cause a collision.");
    }
    {
        // what is the goal of this test? Should it test momentum and collission normal
        glm::vec3 sphere_velocity_initial = glm::vec3(-1.0f, 0.0f, 0.0f);
        engine::physics::Sphere sphere_small_overlap(1.0f, 1.0f, glm::vec3(1.499f, 0.0f, 0.0f),
                                                     sphere_velocity_initial);
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f);
        engine::physics::Collisions::handleElasticCollision(box, sphere_small_overlap);
        TestUtil::ExpectVec3Near(-glm::normalize(sphere_velocity_initial),
                                 glm::normalize(sphere_small_overlap.velocity),
                                 "Barely overlapping sphere should reflect back.");
    }
    {
        glm::vec3 sphere_velocity_initial = glm::vec3(-1.0f, 0.0f, 0.0f);
        glm::vec3 sphere_position_initial = glm::vec3(1.501f, 0.0f, 0.0f);
        engine::physics::Sphere sphere_no_overlap(1.0f, 1.0f, sphere_position_initial,
                                                  sphere_velocity_initial);
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f));
        engine::physics::Collisions::handleElasticCollision(box, sphere_no_overlap);
        TestUtil::ExpectVec3Near(sphere_velocity_initial, sphere_no_overlap.velocity,
                                 "Non-overlapping sphere should not change.");
        TestUtil::ExpectVec3Near(sphere_position_initial, sphere_no_overlap.position,
                                 "Non-overlapping sphere should not change.");
    }
}

// Refactored test using the helper
TEST(CollisionResolutionTest, SphereEdgeAndCornerCases)
{
    // clang-format off
    // --- Symmetrical edge collision ---
    runSphereBoxCollisionTest({
        glm::vec3(std::sqrt(0.5f*0.5f + 0.5f*0.5f), std::sqrt(0.5f*0.5f + 0.5f*0.5f), 0.0f),
        glm::vec3(-1.0f, 0.0f, 0.0f),
        1.0f, 1.0f,
        glm::vec3(0.5f, 0.5f, 0.0f),
        50.0f
    });

    // --- Offset edge collision (30/60/90 triangle) ---
    runSphereBoxCollisionTest({
        glm::vec3((0.5f + 1.0f) - 0.01f, (0.5f + std::sqrt(3.0f)) - 0.01f, 0.0f),
        glm::vec3(-2.0f, 0.0f, 0.0f),
        2.0f, 1.0f,
        glm::vec3(0.5f, 0.5f, 0.0f),
        50.0f
    });

    // --- Symmetrical corner collision ---
    runSphereBoxCollisionTest({
        glm::vec3((0.5f + 1.0f) - 0.01f, (0.5f + std::sqrt(3.0f)) - 0.01f, (0.5f + 1.0f) - 0.01f),
        glm::vec3(-2.0f, 0.0f, -2.0f),
        std::sqrt(5.0f), 1.0f,
        glm::vec3(0.5f, 0.5f, 0.5f),
        50.0f
    });

    // --- Offset corner collision (30/60/90 triangle) ---
    runSphereBoxCollisionTest({
        glm::vec3((0.5f + 1.0f) - 0.01f, (0.5f + std::sqrt(3.0f)) - 0.01f, (0.5f + 1.0f) - 0.01f),
        glm::vec3(-2.0f, 0.0f, -2.0f),
        std::sqrt(5.0f), 1.0f,
        glm::vec3(0.5f),
        50.0f
    });

    // --- Add more cases as needed ---
    // clang-format on
}

// TODO tunneling resolution
