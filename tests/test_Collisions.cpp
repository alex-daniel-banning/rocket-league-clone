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
    glm::vec3 box_vel;
    float box_mass;
    glm::quat box_rotation;
    std::string label;
};

glm::vec3 expectedVelocity(const glm::vec3 &v_self, const glm::vec3 &v_other, float m_self,
                           float m_other, const glm::vec3 &normal)
{
    return v_self -
           (2.0f * m_other / (m_self + m_other)) * glm::dot(v_self - v_other, normal) * normal;
}

// Minimal helper function to run a sphere vs box collision test
void runSphereBoxCollisionResolutionTest(const SphereBoxCase &c)
{
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

    std::string msg = "\nFailure for Sphere v. Box collision resolution. TEST CASE -> " + c.label;
    TestUtil::ExpectVec3Near(expected_sphere_vel, sphere.velocity,
                             msg + "\nSphere velocity:", 1e-6);
    TestUtil::ExpectVec3Near(expected_box_vel, box.velocity, msg + "\nBox velocity:", 1e-6);
}

// Struct for sphere vs box detection test case
struct SphereBoxDetectionCase
{
    glm::vec3 sphere_pos;
    float sphere_radius;
    glm::quat box_rotation;
    bool expect_collision;
    std::string label;
};

// Minimal helper function to run a detection test
void runSphereBoxDetectionTest(const SphereBoxDetectionCase &c)
{
    engine::physics::Sphere sphere(c.sphere_radius, 1.0f, c.sphere_pos, glm::vec3(0.0f));
    engine::physics::Box box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 1.0f,
                             c.box_rotation);

    engine::physics::Contact contact;
    bool collided   = engine::physics::Collisions::computeContact(box, sphere, contact);
    std::string msg = "Failure for Sphere v. Box collision detection. TEST CASE -> " + c.label;
    EXPECT_EQ(c.expect_collision, collided) << msg;
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

TEST(CollisionDetectionTest, SphereVBox)
{
    // clang-format off
    runSphereBoxDetectionTest({
        .sphere_pos = { glm::vec3(1.5f, 0.0f, 0.0f) },
        .sphere_radius = 1.0f,
        .box_rotation = { glm::quat() },
        .expect_collision = false,
        .label = "Sphere just touching face"
    });

    runSphereBoxDetectionTest({
        .sphere_pos = { glm::vec3(1.499f, 0.0f, 0.0f) },
        .sphere_radius = 1.0f,
        .box_rotation = { glm::quat() },
        .expect_collision = true,
        .label = "Sphere barely overlapping face"
    });

    runSphereBoxDetectionTest({
        .sphere_pos = { glm::vec3(0.5f + 1.0f - 0.01f, 0.5f + std::sqrt(3.0f) - 0.01f, 0.0f) },
        .sphere_radius = 2.0f,
        .box_rotation = { glm::quat() },
        .expect_collision = true,
        .label = "Offset edge collision"
    });

    runSphereBoxDetectionTest({
        .sphere_pos = { glm::vec3(0.5f + 1.0f - 0.01f, 0.5f + std::sqrt(3.0f) - 0.01f, 0.5f + 1.0f - 0.01f) },
        .sphere_radius = std::sqrt(5.0f),
        .box_rotation = { glm::quat() },
        .expect_collision = true,
        .label = "Symmetrical corner collision"
    });

    runSphereBoxDetectionTest({
        .sphere_pos = { glm::vec3(2.0f, 2.0f, 2.0f) },
        .sphere_radius = 1.0f,
        .box_rotation = { glm::quat() },
        .expect_collision = false,
        .label = "No overlap collision"
    });

    runSphereBoxDetectionTest({
        .sphere_pos = { glm::vec3((std::sqrt(2 * (0.5f * 0.5f))) + 1.0f - 0.01f, 0.0f, 0.0f) },
        .sphere_radius = 1.0f,
        .box_rotation = { glm::quat(glm::angleAxis(glm::radians(45.0f), glm::vec3(0,1,0))) },
        .expect_collision = true,
        .label = "Sphere touching because of rotation"
    });
    // clang-format on
}

TEST(CollisionResolutionTest, SphereVBox)
{
    // clang-format off
    runSphereBoxCollisionResolutionTest({
        .sphere_pos = { glm::vec3(1.499, 0.0f, 0.0f) },
        .sphere_vel = { glm::vec3(-1.0f, 0.0f, 0.0f) },
        .sphere_radius = 1.0f,
        .sphere_mass = 1.0f,
        .contact_point_approx = { glm::vec3(0.0f) },
        .box_vel = { glm::vec3(0.0f) },
        .box_mass = 1.0f,
        .box_rotation = { glm::quat() },
        .label = "Face collision"
    });

    runSphereBoxCollisionResolutionTest({
        .sphere_pos = { glm::vec3(std::sqrt(0.5f*0.5f + 0.5f*0.5f), std::sqrt(0.5f*0.5f + 0.5f*0.5f), 0.0f) },
        .sphere_vel = { glm::vec3(-1.0f, 0.0f, 0.0f) },
        .sphere_radius = 1.0f,
        .sphere_mass = 1.0f,
        .contact_point_approx = { glm::vec3(0.5f, 0.5f, 0.0f) },
        .box_vel = { glm::vec3(0.0f) },
        .box_mass = 50.0f,
        .box_rotation = { glm::quat() },
        .label = "Symmetrical edge collision"
    });

    runSphereBoxCollisionResolutionTest({
        .sphere_pos = { glm::vec3((0.5f + 1.0f) - 0.01f, (0.5f + std::sqrt(3.0f)) - 0.01f, 0.0f) },
        .sphere_vel = { glm::vec3(-2.0f, 0.0f, 0.0f) },
        .sphere_radius = 2.0f,
        .sphere_mass = 1.0f,
        .contact_point_approx = { glm::vec3(0.5f, 0.5f, 0.0f) },
        .box_vel = { glm::vec3(0.0f) },
        .box_mass = 50.0f,
        .box_rotation = { glm::quat() },
        .label = "Offset edge collision (30/60/90 triangle)"
    });

    runSphereBoxCollisionResolutionTest({
        .sphere_pos = { glm::vec3((0.5f + 1.0f) - 0.01f, (0.5f + std::sqrt(3.0f)) - 0.01f, (0.5f + 1.0f) - 0.01f) },
        .sphere_vel = { glm::vec3(-2.0f, 0.0f, -2.0f) },
        .sphere_radius = std::sqrt(5.0f),
        .sphere_mass = 1.0f,
        .contact_point_approx = { glm::vec3(0.5f, 0.5f, 0.5f) },
        .box_vel = { glm::vec3(0.0f) },
        .box_mass = 50.0f,
        .box_rotation = { glm::quat() },
        .label = "Symmetrical corner collision"
    });

    runSphereBoxCollisionResolutionTest({
        .sphere_pos = { glm::vec3((0.5f + 1.0f) - 0.01f, (0.5f + std::sqrt(3.0f)) - 0.01f, (0.5f + 1.0f) - 0.01f) },
        .sphere_vel = { glm::vec3(-2.0f, 0.0f, -2.0f) },
        .sphere_radius = std::sqrt(5.0f),
        .sphere_mass = 1.0f,
        .contact_point_approx = { glm::vec3(0.5f) },
        .box_vel = { glm::vec3(0.0f) },
        .box_mass = 50.0f,
        .box_rotation = { glm::quat() },
        .label = "Offset corner collision (30/60/90 triangle)"
    });

    // --- Add more cases as needed ---
    // clang-format on
}

// TODO tunneling resolution
