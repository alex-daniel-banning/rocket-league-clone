#include <engine/physics/collisions.hpp>
#include <gtest/gtest.h>

#include "test_util.hpp"

// Base fixture with common utilities
class CollisionResolutionTest : public ::testing::Test {
 protected:
  // Helper to compute total linear momentum
  glm::vec3 TotalLinearMomentum(const engine::physics::Sphere& s, const engine::physics::Box& b) {
    return s.mass * s.velocity + b.mass * b.velocity;
  }

  // Helper to compute total angular momentum
  glm::vec3 TotalAngularMomentum(const engine::physics::Sphere& s, const engine::physics::Box& b) {
    // Orbital angular momentum: L_orbital = r x p
    glm::vec3 sphere_orbital_momentum = glm::cross(s.position, s.mass * s.velocity);
    glm::vec3 box_orbital_momentum = glm::cross(b.position, b.mass * b.velocity);

    // Spin angular momentum: L_spin = I * w
    glm::vec3 sphere_spin_momentum = glm::vec3(0.0f);  // Not implemented yet (no friction)
    glm::vec3 box_spin_momentum = b.inertia_tensor * b.angular_velocity;

    return sphere_orbital_momentum + box_orbital_momentum + sphere_spin_momentum + box_spin_momentum;
  }

  // Helper to compute total kinetic energy
  float TotalKineticEnergy(const engine::physics::Sphere& s, const engine::physics::Box& b) {
    float linear = 0.5f * s.mass * glm::dot(s.velocity, s.velocity) + 0.5f * b.mass * glm::dot(b.velocity, b.velocity);
    // Only box has rotational KE: KE_rot = (1/2) * w^T * I * w
    glm::vec3 moment_of_inertia = b.inertia_tensor * b.angular_velocity;
    float rotational = 0.5f * glm::dot(b.angular_velocity, moment_of_inertia);
    return linear + rotational;
  }
};

// Conservation law tests - these could be parameterized across scenarios
class SphereBox : public CollisionResolutionTest, public ::testing::WithParamInterface<std::string> {
 protected:
  // Different collision scenarios
  std::pair<engine::physics::Sphere, engine::physics::Box> SetupScenario(const std::string& scenario) {
    if (scenario == "Approaching") {
      return {engine::physics::Sphere(1.0f, 2.0f, glm::vec3(1.499f, 0.0f, 0.0f), glm::vec3(-2.0f, 0.0f, 0.0f)),
              engine::physics::Box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f), 3.0f, glm::quat())};
    } else if (scenario == "SameDirection") {
      return {engine::physics::Sphere(1.0f, 2.0f, glm::vec3(1.499f, 0.0f, 0.0f), glm::vec3(-3.0f, 0.0f, 0.0f)),
              engine::physics::Box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), 3.0f, glm::quat())};
    } else if (scenario == "OffCenter") {
      return {engine::physics::Sphere(1.0f, 1.0f, glm::vec3(1.499f, 0.3f, 0.0f), glm::vec3(-2.0f, 0.0f, 0.0f)),
              engine::physics::Box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f, glm::quat())};
    } else {
      throw std::invalid_argument("Unknown scenario: " + scenario);
    }
  }
};

TEST_P(SphereBox, LinearMomentum) {
  auto [sphere, box] = SetupScenario(GetParam());

  glm::vec3 momentum_before = TotalLinearMomentum(sphere, box);

  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box, sphere, contact));
  engine::physics::Collisions::ResolveElasticCollision(box, sphere, contact);

  glm::vec3 momentum_after = TotalLinearMomentum(sphere, box);

  TestUtil::ExpectVec3Near(momentum_before, momentum_after, "Linear momentum", 1e-5f);
}

TEST_P(SphereBox, AngularMomentum) {
  auto [sphere, box] = SetupScenario(GetParam());

  glm::vec3 momentum_before = TotalAngularMomentum(sphere, box);

  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box, sphere, contact));
  engine::physics::Collisions::ResolveElasticCollision(box, sphere, contact);

  glm::vec3 momentum_after = TotalAngularMomentum(sphere, box);

  TestUtil::ExpectVec3Near(momentum_before, momentum_after, "Angular momentum", 1e-5f);
}

TEST_P(SphereBox, TotalKineticEnergy) {
  auto [sphere, box] = SetupScenario(GetParam());

  float ke_before = TotalKineticEnergy(sphere, box);

  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box, sphere, contact));
  engine::physics::Collisions::ResolveElasticCollision(box, sphere, contact);

  float ke_after = TotalKineticEnergy(sphere, box);

  EXPECT_NEAR(ke_before, ke_after, 1e-4f) << "Total kinetic energy should be conserved in elastic collision";
}

INSTANTIATE_TEST_SUITE_P(ConservationLaws, SphereBox, ::testing::Values("Approaching", "SameDirection", "OffCenter"),
                         [](const testing::TestParamInfo<std::string>& info) { return info.param; });
