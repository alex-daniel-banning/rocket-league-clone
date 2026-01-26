#include <engine/physics/collisions.hpp>
#include <gtest/gtest.h>

#include "test_util.hpp"

TEST(SpherePlane, DetectsCollision) {
  glm::vec3 color(0.2f, 0.15f, 0.15f);
  float plane_length = 10.0f;
  glm::vec3 plane_initial_position(0.0f);
  glm::quat plane_rotation(glm::vec3(0.0f));
  engine::physics::Plane plane(plane_length, plane_length, color,
                               plane_initial_position, plane_rotation);
  ASSERT_EQ(glm::vec3(0.0f, 1.0f, 0.0f), plane.GetNormal());

  engine::physics::Sphere sphere;
  sphere.radius = 1.0f;
  sphere.position = glm::vec3(2.5f, sphere.radius / 2, 2.5f);
  sphere.velocity = glm::vec3(0.0f, 0.0f, 0.0f);

  EXPECT_EQ(true, engine::physics::Collisions::Collides(plane, sphere));

  sphere.position = glm::vec3(2.5f, sphere.radius, 2.5f);
  EXPECT_EQ(false, engine::physics::Collisions::Collides(plane, sphere));

  sphere.position = glm::vec3(2.5f, sphere.radius + 0.01f, 2.5f);
  EXPECT_EQ(false, engine::physics::Collisions::Collides(plane, sphere));
}

TEST(SpherePlane, ResolvesElasticCollision) {
  glm::vec3 color(0.2f, 0.15f, 0.15f);
  float plane_length = 10.0f;
  glm::vec3 plane_initial_position(0.0f);
  glm::quat plane_rotation(glm::vec3(0.0f));
  engine::physics::Plane plane(plane_length, plane_length, color,
                               plane_initial_position, plane_rotation);
  ASSERT_EQ(glm::vec3(0.0f, 1.0f, 0.0f), plane.GetNormal());

  engine::physics::Sphere sphere;
  sphere.radius = 1.0f;
  sphere.position = glm::vec3(2.5f, 0.5, 2.5f);
  sphere.velocity = glm::vec3(0.0f, -1.0f, 0.0f);

  engine::physics::Collisions::HandleElasticCollision(plane, sphere);

  EXPECT_EQ(glm::vec3(0.0f, 1.0f, 0.0f), sphere.velocity);
}

TEST(SpherePlane, ResolvesCornerCollision) {
  glm::vec3 color(0.2f, 0.15f, 0.15f);
  float plane_length = 10.0f;
  engine::physics::Plane plane1(plane_length, plane_length, color,
                                glm::vec3(0.0f), glm::quat());
  TestUtil::AssertVec3Near(glm::vec3(0.0f, 1.0f, 0.0f), plane1.GetNormal(),
                           "Unexpected normal for initialized plane.");
  engine::physics::Plane plane2(
      plane_length, plane_length, color,
      glm::vec3(0.0f, plane_length / 2, -plane_length / 2),
      glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
  TestUtil::AssertVec3Near(glm::vec3(0.0f, 0.0f, 1.0f), plane2.GetNormal(),
                           "Unexpected normal for initialized plane.");

  engine::physics::Sphere sphere;
  sphere.radius = 1.0f;
  sphere.position = glm::vec3(0.0f, 0.5f, -4.5f);
  sphere.velocity = glm::vec3(0.0f, -1.0f, -1.0f);

  engine::physics::Collisions::HandleElasticCollision(plane1, sphere);
  engine::physics::Collisions::HandleElasticCollision(plane2, sphere);

  TestUtil::ExpectVec3Near(
      glm::vec3(0.0f, 1.0f, 1.0f), sphere.velocity,
      "Sphere did not handle two collisions in one frame as expected.");
}

TEST(SpherePlane, ResolvesZeroVelocityPenetration) {
  // Sphere is inside of sphere with 0 velocity
  // After collision, sphere should be telleported to "good side" (normal
  // direction) of plane so that sphere is just touching the plane. This means
  // that this collision resolution should only apply when the sphere position +
  // radius is greater than distance from plane, "not greater than or equal."
  glm::vec3 color(0.2f, 0.15f, 0.15f);
  float plane_length = 10.0f;
  glm::vec3 plane_initial_position(0.0f);
  glm::quat plane_rotation(glm::vec3(0.0f));
  engine::physics::Plane plane(plane_length, plane_length, color,
                               plane_initial_position, plane_rotation);
  TestUtil::AssertVec3Near(glm::vec3(0.0f, 1.0f, 0.0f), plane.GetNormal(),
                           "Unexpected normal for initialized plane.");

  engine::physics::Sphere sphere;
  sphere.radius = 1.0f;
  sphere.position = glm::vec3(2.5f, (sphere.radius / 2), 2.5f);
  sphere.velocity = glm::vec3(0.0f, 0.0f, 0.0f);

  ASSERT_EQ(true, engine::physics::Collisions::Collides(plane, sphere));
  engine::physics::Collisions::HandleElasticCollision(plane, sphere);
  TestUtil::ExpectVec3Near(
      glm::vec3(2.5f, sphere.radius, 2.5f), sphere.position,
      "Sphere shouldn't overlap with plane when it isn't moving.");
  EXPECT_EQ(glm::vec3(0.0f), sphere.velocity)
      << "Sphere shouldn't be accelerated by zero-velocity overlap.";
}

TEST(SpherePlane, ResolvesParallelVelocityPenetration) {
  // Sphere is inside of sphere with 0 velocity
  // After collision, sphere should be telleported to "good side" (normal
  // direction) of plane so that sphere is just touching the plane. This means
  // that this collision resolution should only apply when the sphere position +
  // radius is greater than distance from plane, "not greater than or equal."
  glm::vec3 color(0.2f, 0.15f, 0.15f);
  float plane_length = 10.0f;
  glm::vec3 plane_initial_position(0.0f);
  glm::quat plane_rotation(glm::vec3(0.0f));
  engine::physics::Plane plane(plane_length, plane_length, color,
                               plane_initial_position, plane_rotation);
  TestUtil::AssertVec3Near(glm::vec3(0.0f, 1.0f, 0.0f), plane.GetNormal(),
                           "Unexpected normal for initialized plane.");

  engine::physics::Sphere sphere;
  sphere.radius = 1.0f;
  sphere.position = glm::vec3(2.5f, (sphere.radius / 2), 2.5f);
  sphere.velocity = glm::vec3(0.0f, 0.0f, -1.0f);

  ASSERT_EQ(true, engine::physics::Collisions::Collides(plane, sphere));
  engine::physics::Collisions::HandleElasticCollision(plane, sphere);
  TestUtil::ExpectVec3Near(glm::vec3(2.5f, sphere.radius, 2.5f),
                           sphere.position);
  EXPECT_EQ(glm::vec3(0.0f, 0.0f, -1.0f), sphere.velocity)
      << "Overlap resultion shouldn't add velocity.";
}

// Parameterized test for sphere vs box collision detection
