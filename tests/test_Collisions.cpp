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
    TestUtil::AssertVec3Near(glm::vec3(0.0f, 1.0f, 0.0f), plane1.getNormal());
    engine::physics::Plane plane2(planeLength, planeLength, color,
                                  glm::vec3(0.0f, planeLength / 2, -planeLength / 2),
                                  glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
    TestUtil::AssertVec3Near(glm::vec3(0.0f, 0.0f, 1.0f), plane2.getNormal());

    engine::physics::Sphere sphere;
    sphere.radius   = 1.0f;
    sphere.position = glm::vec3(0.0f, 0.5f, -4.5f);
    sphere.velocity = glm::vec3(0.0f, -1.0f, -1.0f);

    engine::physics::Collisions::handleElasticCollision(plane1, sphere);
    engine::physics::Collisions::handleElasticCollision(plane2, sphere);

    TestUtil::ExpectVec3Near(glm::vec3(0.0f, 1.0f, 1.0f), sphere.velocity, 1e-5f);
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
    TestUtil::AssertVec3Near(glm::vec3(0.0f, 1.0f, 0.0f), plane.getNormal());

    engine::physics::Sphere sphere;
    sphere.radius   = 1.0f;
    sphere.position = glm::vec3(2.5f, (sphere.radius / 2), 2.5f);
    sphere.velocity = glm::vec3(0.0f, 0.0f, 0.0f);

    ASSERT_EQ(true, engine::physics::Collisions::collides(plane, sphere));
    engine::physics::Collisions::handleElasticCollision(plane, sphere);
    TestUtil::ExpectVec3Near(glm::vec3(2.5f, sphere.radius, 2.5f), sphere.position);
    EXPECT_EQ(glm::vec3(0.0f), sphere.velocity);
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
    TestUtil::AssertVec3Near(glm::vec3(0.0f, 1.0f, 0.0f), plane.getNormal());

    engine::physics::Sphere sphere;
    sphere.radius   = 1.0f;
    sphere.position = glm::vec3(2.5f, (sphere.radius / 2), 2.5f);
    sphere.velocity = glm::vec3(0.0f, 0.0f, -1.0f);

    ASSERT_EQ(true, engine::physics::Collisions::collides(plane, sphere));
    engine::physics::Collisions::handleElasticCollision(plane, sphere);
    TestUtil::ExpectVec3Near(glm::vec3(2.5f, sphere.radius, 2.5f), sphere.position);
    EXPECT_EQ(glm::vec3(0.0f, 0.0f, -1.0f), sphere.velocity);
}

// TODO tunneling resolution
