#include <engine/physics/box_builder.hpp>
#include <engine/physics/collisions.hpp>
#include <gtest/gtest.h>

// --- Sphere vs Box friction tests ---

// Sphere hits the +X face of an immovable 2x2x2 box while sliding in +Z.
// Sphere position (1.999, 0, 0) with radius 1 just barely penetrates the face.

TEST(Friction_SphereBox, ZeroFriction_TangentialVelocityUnchanged) {
  engine::physics::Box box =
      engine::physics::BoxBuilder().Size(glm::vec3(2.0f)).Mass(0.0f).Build();
  engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.999f, 0.0f, 0.0f), glm::vec3(-2.0f, 0.0f, 3.0f));

  float v_z_before = sphere.velocity.z;
  engine::physics::Collisions::HandleCollision(box, sphere, 0.0f, 0.0f);

  EXPECT_NEAR(sphere.velocity.z, v_z_before, 1e-4f) << "Tangential velocity should be unchanged with friction=0";
}

TEST(Friction_SphereBox, NonZeroFriction_TangentialVelocityReduced) {
  engine::physics::Box box =
      engine::physics::BoxBuilder().Size(glm::vec3(2.0f)).Mass(0.0f).Build();
  engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.999f, 0.0f, 0.0f), glm::vec3(-2.0f, 0.0f, 3.0f));

  float v_z_before = sphere.velocity.z;
  engine::physics::Collisions::HandleCollision(box, sphere, 0.0f, 0.5f);

  EXPECT_LT(sphere.velocity.z, v_z_before) << "Friction should reduce tangential velocity";
  EXPECT_GT(sphere.velocity.z, 0.0f) << "Tangential velocity should remain positive, not reversed";
}

TEST(Friction_SphereBox, HigherFriction_MoreTangentialReduction) {
  auto run = [](float mu) {
    engine::physics::Box box =
        engine::physics::BoxBuilder().Size(glm::vec3(2.0f)).Mass(0.0f).Build();
    engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.999f, 0.0f, 0.0f), glm::vec3(-2.0f, 0.0f, 3.0f));
    engine::physics::Collisions::HandleCollision(box, sphere, 0.0f, mu);
    return sphere.velocity.z;
  };

  EXPECT_LT(run(0.9f), run(0.1f)) << "Higher friction should reduce tangential velocity more";
}

// Sphere slides in +Z along the +X face of a movable box.
// The friction torque at the contact point should spin the box about -Y.
TEST(Friction_SphereBox, Friction_GeneratesBoxAngularVelocity) {
  engine::physics::Box box =
      engine::physics::BoxBuilder().Size(glm::vec3(1.0f)).Mass(5.0f).Build();
  engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.499f, 0.0f, 0.0f), glm::vec3(-2.0f, 0.0f, 3.0f));

  engine::physics::Collisions::HandleCollision(box, sphere, 0.0f, 1.0f);

  // Contact at (0.5, 0, 0) relative to box center. Friction on box is in +Z.
  // cross((0.5,0,0), (0,0,+Z)) = (0, -0.5*Z, 0) → angular_velocity.y < 0
  EXPECT_LT(box.angular_velocity.y, 0.0f) << "Sliding sphere should torque box about -Y";
  EXPECT_NEAR(box.angular_velocity.x, 0.0f, 1e-4f);
  EXPECT_NEAR(box.angular_velocity.z, 0.0f, 1e-4f);
}

// Large tangential velocity, small normal velocity: the Coulomb clamp prevents
// friction from fully arresting the slide. Tangential speed should be barely
// reduced rather than zeroed.
TEST(Friction_SphereBox, CoulombClamp_LimitsFrictionImpulse) {
  engine::physics::Box box =
      engine::physics::BoxBuilder().Size(glm::vec3(2.0f)).Mass(0.0f).Build();
  // v_normal = 0.1 (below bounce threshold → e=0, j_n ≈ 0.1)
  // v_tangential = 10.0 (large)
  // mu = 0.5 → friction bound = 0.5 * 0.1 = 0.05
  // Unclamped friction would need j=-10; clamped to -0.05 → Δv_z ≈ -0.05
  engine::physics::Sphere sphere(1.0f, 1.0f, glm::vec3(1.999f, 0.0f, 0.0f), glm::vec3(-0.1f, 0.0f, 10.0f));

  engine::physics::Collisions::HandleCollision(box, sphere, 0.0f, 0.5f);

  EXPECT_GT(sphere.velocity.z, 9.0f)
      << "Coulomb clamp should prevent friction from significantly reducing large tangential velocity";
}

// --- Box vs Box friction tests ---

// Box A stationary; Box B moving toward A (−X) and sliding in +Z.
// Normal = +X (from A to B). Tangential component is Z.

TEST(Friction_BoxBox, ZeroFriction_TangentialVelocityUnchanged) {
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
                   .Position(0.999f, 0.0f, 0.0f)
                   .Velocity(glm::vec3(-1.0f, 0.0f, 3.0f))
                   .Build();

  float v_z_b_before = box_b.velocity.z;
  float v_z_a_before = box_a.velocity.z;
  engine::physics::Collisions::HandleCollision(box_a, box_b, 0.0f, 0.0f);

  EXPECT_NEAR(box_b.velocity.z, v_z_b_before, 1e-4f) << "Box B tangential velocity should be unchanged with friction=0";
  EXPECT_NEAR(box_a.velocity.z, v_z_a_before, 1e-4f) << "Box A tangential velocity should be unchanged with friction=0";
}

TEST(Friction_BoxBox, NonZeroFriction_TangentialVelocityReduced) {
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
                   .Position(0.999f, 0.0f, 0.0f)
                   .Velocity(glm::vec3(-1.0f, 0.0f, 3.0f))
                   .Build();

  float v_z_b_before = box_b.velocity.z;
  engine::physics::Collisions::HandleCollision(box_a, box_b, 0.0f, 0.5f);

  EXPECT_LT(box_b.velocity.z, v_z_b_before) << "Friction should reduce Box B's tangential velocity";
  EXPECT_GT(box_a.velocity.z, 0.0f) << "Friction should transfer tangential velocity to Box A";
}
