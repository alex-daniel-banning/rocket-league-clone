#include "engine/physics/contact_constraint_solver.hpp"

#include <gtest/gtest.h>

#include "engine/physics/box.hpp"
#include "engine/physics/box_builder.hpp"
#include "engine/physics/collisions.hpp"
#include "engine/physics/contact.hpp"

namespace engine::physics {

namespace {

Contact MakeContact(int id_a, int id_b, glm::vec3 normal, std::vector<ContactPoint> points) {
  Contact c;
  c.body_a_id = id_a;
  c.body_b_id = id_b;
  c.normal = normal;
  c.points = std::move(points);
  c.penetration = c.points.empty() ? 0.0f : c.points[0].penetration;
  return c;
}

struct SolverResult {
  std::unordered_map<int, Body> bodies;
};

SolverResult RunSolver(Box& a, Box& b, const Contact& contact, float restitution = 1.0f, float baumgarte = 0.0f,
                       int iterations = 10) {
  std::unordered_map<int, Body> bodies;
  bodies[a.GetId()] = &a;
  bodies[b.GetId()] = &b;
  std::vector<ContactConstraint> constraints;
  float dt = 1.0f / 120.0f;
  ContactConstraintSolver::GenerateFromContact(contact, bodies, dt, constraints, restitution, baumgarte);
  ContactConstraintSolver::Solve(bodies, constraints, iterations);
  return {bodies};
}

}  // namespace

TEST(BoxBoxResolution, HeadOnCollision_ProducesNoRotation) {
  Box top = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -1, 0).Id(1).Build();
  Box bot = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  RunSolver(top, bot, contact);

  float eps = 1e-4f;
  EXPECT_NEAR(top.velocity.x, 0.0f, eps);
  EXPECT_NEAR(top.velocity.y, 1.0f, eps);
  EXPECT_NEAR(top.velocity.z, 0.0f, eps);
  EXPECT_NEAR(bot.velocity.x, 0.0f, eps);
  EXPECT_NEAR(bot.velocity.y, -1.0f, eps);
  EXPECT_NEAR(bot.velocity.z, 0.0f, eps);
  EXPECT_NEAR(glm::length(top.angular_velocity), 0.0f, eps);
  EXPECT_NEAR(glm::length(bot.angular_velocity), 0.0f, eps);
}

TEST(BoxBoxResolution, LighterBoxMovesFaster) {
  Box light = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(1).Velocity(0, -1, 0).Id(1).Build();
  Box heavy = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(3).Velocity(0, 1, 0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  RunSolver(light, heavy, contact);

  EXPECT_GT(std::abs(light.velocity.y), std::abs(heavy.velocity.y));
}

TEST(BoxBoxResolution, SameDirectionCollision) {
  Box fast = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -3, 0).Id(1).Build();
  Box slow = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, -1, 0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  RunSolver(fast, slow, contact);

  float eps = 1e-4f;
  // Equal mass elastic: velocities swap
  EXPECT_NEAR(fast.velocity.y, -1.0f, eps);
  EXPECT_NEAR(slow.velocity.y, -3.0f, eps);
}

TEST(BoxBoxResolution, ZeroRelativeVelocity_ShouldCreateNoImpulse) {
  Box a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -1, 0).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, -1, 0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  // Baumgarte stabilization disabled by default. Otherwise, this would apply a corrective impulse.
  RunSolver(a, b, contact);

  float eps = 1e-4f;
  EXPECT_NEAR(a.velocity.y, -1.0f, eps);
  EXPECT_NEAR(b.velocity.y, -1.0f, eps);
  EXPECT_NEAR(glm::length(a.angular_velocity), 0.0f, eps);
  EXPECT_NEAR(glm::length(b.angular_velocity), 0.0f, eps);
}

TEST(BoxBoxResolution, OffCenterImpact_ProducesAngularVelocity) {
  Box a = BoxBuilder().Size(2.0f).Position(0.5f, 0.99f, 0).Mass(10).Velocity(0, -1, 0).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 0, 0).Id(2).Build();

  Contact contact;
  ASSERT_TRUE(Collisions::ComputeContact(a, b, contact));

  RunSolver(a, b, contact);

  EXPECT_GT(glm::length(a.angular_velocity), 1e-4f);
  EXPECT_GT(glm::length(b.angular_velocity), 1e-4f);
}

TEST(BoxBoxResolution, RotationOnlyCollision) {
  Box spinner = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).AngularVelocity(0, 0, 5).Id(1).Build();
  Box stationary = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Id(2).Build();

  Contact contact;
  ASSERT_TRUE(Collisions::ComputeContact(spinner, stationary, contact));

  float spin_before = glm::length(spinner.angular_velocity);
  float vel_before = glm::length(stationary.velocity);

  RunSolver(spinner, stationary, contact);

  // Spinner should lose some angular velocity
  EXPECT_LT(glm::length(spinner.angular_velocity), spin_before);
  // Stationary box should gain some linear velocity
  EXPECT_GT(glm::length(stationary.velocity), vel_before + 1e-4f);
}

}  // namespace engine::physics
