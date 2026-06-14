#include <gtest/gtest.h>

#include "engine/physics/box.hpp"
#include "engine/physics/box_builder.hpp"
#include "engine/physics/constraint_solver.hpp"
#include "engine/physics/contact.hpp"
#include "engine/physics/friction_constraint.hpp"
#include "engine/physics/sphere.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

namespace engine::physics {

namespace {

Contact MakeContact(int id_a, int id_b, glm::vec3 normal, std::vector<ContactPoint> points) {
  Contact c;
  c.body_a_id = id_a;
  c.body_b_id = id_b;
  c.normal = normal;
  c.points = std::move(points);
  return c;
}

struct SolverResult {
  std::unordered_map<int, Body> bodies;
  std::vector<NormalConstraint> normal_constraints;
  std::vector<FrictionConstraint> friction_constraints;
};

SolverResult RunSolver(Box& a, Box& b, const Contact& contact, float restitution = 1.0f, float baumgarte = 0.0f,
                       int iterations = 10) {
  std::unordered_map<int, Body> bodies;
  bodies[a.GetId()] = &a;
  bodies[b.GetId()] = &b;
  std::vector<NormalConstraint> normal_constraints;
  std::vector<FrictionConstraint> friction_constraints;
  float dt = 1.0f / 120.0f;
  ConstraintSolver::GenerateFromContact(contact, bodies, dt, normal_constraints, friction_constraints, restitution,
                                        baumgarte);
  ConstraintSolver solver;
  solver.Solve(bodies, normal_constraints, friction_constraints, iterations);
  return {bodies, normal_constraints, friction_constraints};
}

SolverResult RunSolver(Sphere& a, int id_a, Box& b, const Contact& contact, float restitution = 1.0f,
                       float baumgarte = 0.0f, int iterations = 10) {
  std::unordered_map<int, Body> bodies;
  bodies[id_a] = &a;
  bodies[b.GetId()] = &b;
  std::vector<NormalConstraint> normal_constraints;
  std::vector<FrictionConstraint> friction_constraints;
  float dt = 1.0f / 120.0f;
  ConstraintSolver::GenerateFromContact(contact, bodies, dt, normal_constraints, friction_constraints, restitution,
                                        baumgarte);
  ConstraintSolver solver;
  solver.Solve(bodies, normal_constraints, friction_constraints, iterations);
  return {bodies, normal_constraints, friction_constraints};
}

}  // namespace

TEST(Friction, SlidingDeceleration) {
  Box slider = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(5, 0, 0).Friction(0.5f).Id(1).Build();
  Box ground = BoxBuilder().Size(glm::vec3(20, 2, 20)).Position(0, -1, 0).Mass(0).Friction(0.8f).Id(2).Build();
  glm::vec3 contact_point(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{contact_point, 0.01f}});

  RunSolver(slider, ground, contact, 0.0f, 0.2f);

  EXPECT_LT(std::abs(slider.velocity.x), 5.0f);
}

TEST(Friction, FrictionlessSlides) {
  Box slider = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(5, 0, 0).Friction(0).Id(1).Build();
  Box ground = BoxBuilder().Size(glm::vec3(20, 2, 20)).Position(0, -1, 0).Mass(0).Friction(0.8f).Id(2).Build();
  glm::vec3 contact_point(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{contact_point, 0.01f}});

  RunSolver(slider, ground, contact, 0.0f, 0.2f);

  EXPECT_NEAR(slider.velocity.x, 5.0f, 1e-4f);
}

TEST(Friction, HigherFrictionDeceleratesMore) {
  // Low friction
  Box slider_lo = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(5, 0, 0).Friction(0.2f).Id(1).Build();
  Box ground_lo = BoxBuilder().Size(glm::vec3(20, 2, 20)).Position(0, -1, 0).Mass(0).Friction(0.2f).Id(2).Build();
  Contact contact_lo = MakeContact(1, 2, glm::vec3(0, -1, 0), {{glm::vec3(0, 0, 0), 0.01f}});
  RunSolver(slider_lo, ground_lo, contact_lo, 0.0f, 0.2f);

  // High friction
  Box slider_hi = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(5, 0, 0).Friction(0.8f).Id(1).Build();
  Box ground_hi = BoxBuilder().Size(glm::vec3(20, 2, 20)).Position(0, -1, 0).Mass(0).Friction(0.8f).Id(2).Build();
  Contact contact_hi = MakeContact(1, 2, glm::vec3(0, -1, 0), {{glm::vec3(0, 0, 0), 0.01f}});
  RunSolver(slider_hi, ground_hi, contact_hi, 0.0f, 0.2f);

  EXPECT_LT(std::abs(slider_hi.velocity.x), std::abs(slider_lo.velocity.x));
}

TEST(Friction, SphereDevelopsSpin) {
  Sphere sphere(1.0f, 10.0f, glm::vec3(0, 1.99f, 0), glm::vec3(5, 0, 0), glm::quat(), glm::vec3(), 0.5f);
  Box ground = BoxBuilder().Size(glm::vec3(20, 2, 20)).Position(0, 0, 0).Mass(0).Friction(0.8f).Id(2).Build();
  glm::vec3 contact_point(0, 1.0f, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{contact_point, 0.01f}});

  RunSolver(sphere, 1, ground, contact, 0.0f, 0.2f);

  EXPECT_GT(glm::length(sphere.angular_velocity), 1e-4f);
}

TEST(Friction, NoSpinWithoutFriction) {
  Sphere sphere(1.0f, 10.0f, glm::vec3(0, 1.99f, 0), glm::vec3(5, 0, 0), glm::quat(), glm::vec3(), 0.0f);
  Box ground = BoxBuilder().Size(glm::vec3(20, 2, 20)).Position(0, 0, 0).Mass(0).Friction(0.8f).Id(2).Build();
  glm::vec3 contact_point(0, 1.0f, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{contact_point, 0.01f}});

  RunSolver(sphere, 1, ground, contact, 0.0f, 0.2f);

  EXPECT_NEAR(glm::length(sphere.angular_velocity), 0.0f, 1e-6f);
}

TEST(Friction, CoulombBound) {
  Box slider = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(50, 0, 0).Friction(0.3f).Id(1).Build();
  Box ground = BoxBuilder().Size(glm::vec3(20, 2, 20)).Position(0, -1, 0).Mass(0).Friction(0.3f).Id(2).Build();
  glm::vec3 contact_point(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{contact_point, 0.01f}});

  auto result = RunSolver(slider, ground, contact, 0.0f, 0.2f);

  float mu = std::sqrt(0.3f * 0.3f);
  for (const auto& fc : result.friction_constraints) {
    const auto& nc = result.normal_constraints[fc.normal_constraint_index];
    float max_friction = mu * (nc.accumulated_impulse + nc.pseudo_accumulated_impulse);
    EXPECT_LE(std::abs(fc.accumulated_impulse), max_friction + 1e-6f);
  }
}

TEST(Friction, MomentumConservation) {
  Box a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(5).Velocity(3, -1, 2).Friction(0.5f).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(-1, 1, 0).Friction(0.5f).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  glm::vec3 momentum_before = a.mass * a.velocity + b.mass * b.velocity;

  RunSolver(a, b, contact);

  glm::vec3 momentum_after = a.mass * a.velocity + b.mass * b.velocity;
  float eps = 1e-3f;
  EXPECT_NEAR(momentum_before.x, momentum_after.x, eps);
  EXPECT_NEAR(momentum_before.y, momentum_after.y, eps);
  EXPECT_NEAR(momentum_before.z, momentum_after.z, eps);
}

TEST(Friction, NoFrictionConstraintsWhenMuZero) {
  Box a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(5, -1, 0).Friction(0).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Friction(0.5f).Id(2).Build();
  glm::vec3 contact_point(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{contact_point, 0.02f}});

  auto result = RunSolver(a, b, contact);

  EXPECT_TRUE(result.friction_constraints.empty());
}

}  // namespace engine::physics
