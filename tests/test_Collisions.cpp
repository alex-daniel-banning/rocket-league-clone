#include <engine/physics/Collisions.hpp>
#include <gtest/gtest.h>
#include "TestUtil.hpp"

TEST(CollisionTest, BasicCollisionDetection)
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

TEST(CollisionTest, ElasticSphereAssymetricalCornerCollision) {}

TEST(CollisionTest, NoVelocityColissionResolution)
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

TEST(CollisionTest, ParallelVelocityColissionResolution)
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

TEST(CollisionTest, SphereVsBoxFace)
{
    {
        glm::vec3 sphere_velocity_initial = glm::vec3(-1.0f, 0.0f, 0.0f);
        engine::physics::Sphere sphereJustTouching(1.0f, glm::vec3(1.5f, 0.0f, 0.0f),
                                                   sphere_velocity_initial);
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f));
        engine::physics::Collisions::handleElasticCollision(box, sphereJustTouching);
        TestUtil::ExpectVec3Near(
            sphere_velocity_initial, sphereJustTouching.velocity,
            "Sphere that just touches the surface should not cause a collision.");
    }
    {
        glm::vec3 sphere_velocity_initial = glm::vec3(-1.0f, 0.0f, 0.0f);
        engine::physics::Sphere sphere_small_overlap(1.0f, glm::vec3(1.499f, 0.0f, 0.0f),
                                                     sphere_velocity_initial);
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f));
        engine::physics::Collisions::handleElasticCollision(box, sphere_small_overlap);
        TestUtil::ExpectVec3Near(-sphere_velocity_initial, sphere_small_overlap.velocity,
                                 "Barely overlapping sphere should reflect back.");
    }
    {
        glm::vec3 sphere_velocity_initial = glm::vec3(-1.0f, 0.0f, 0.0f);
        glm::vec3 sphere_position_initial = glm::vec3(1.501f, 0.0f, 0.0f);
        engine::physics::Sphere sphere_no_overlap(1.0f, sphere_position_initial,
                                                  sphere_velocity_initial);
        engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f));
        engine::physics::Collisions::handleElasticCollision(box, sphere_no_overlap);
        TestUtil::ExpectVec3Near(sphere_velocity_initial, sphere_no_overlap.velocity,
                                 "Non-overlapping sphere should not change.");
        TestUtil::ExpectVec3Near(sphere_position_initial, sphere_no_overlap.position,
                                 "Non-overlapping sphere should not change.");
    }
}

TEST(CollisionTest, SphereVsBoxEdge)
{
    glm::vec3 sphere_velocity_initial = glm::vec3(-1.0f, 0.0f, 0.0f);

    float x                           = std::sqrt((0.5f * 0.5f) + (0.5f * 0.5f));
    float y                           = std::sqrt((0.5f * 0.5f) + (0.5f * 0.5f));
    glm::vec3 sphere_position_initial = glm::vec3(x, y, 0.0f);
    engine::physics::Sphere sphere(1.0f, sphere_position_initial, sphere_velocity_initial);
    engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f));

    engine::physics::Collisions::handleElasticCollision(box, sphere);

    float new_vel_x = -std::sqrt(0.0f);
    float new_vel_y = std::sqrt(1.0f);
    TestUtil::ExpectVec3Near(glm::vec3(new_vel_x, new_vel_y, 0.0f), sphere.velocity,
                             "Collision is reflected across edge (45 degree collision normal).");
}

TEST(CollisionTest, SphereVsBoxEdgeOffset)
{
    float vel_initial_x               = -2.0f;
    glm::vec3 sphere_velocity_initial = glm::vec3(vel_initial_x, 0.0f, 0.0f);

    // Uses 30/60/90 triangle. 0.5 is offset of corner edge (edge is not at origin).
    float x = (0.5f + 1.0f) - 0.01f;
    float y = (0.5f + std::sqrt(3.0f)) - 0.01f;

    float radius = 2.0f;

    glm::vec3 sphere_position_initial = glm::vec3(x, y, 0.0f);
    engine::physics::Sphere sphere(radius, sphere_position_initial, sphere_velocity_initial);
    engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f));

    engine::physics::Collisions::handleElasticCollision(box, sphere);

    float new_vel_x = -1.0f;
    float new_vel_y = std::sqrt(3.0f);
    TestUtil::ExpectVec3Near(glm::vec3(new_vel_x, new_vel_y, 0.0f), sphere.velocity,
                             "Collision is reflected across edge (60 degree collision normal).",
                             1e-2);
}

// TODO tunneling resolution
