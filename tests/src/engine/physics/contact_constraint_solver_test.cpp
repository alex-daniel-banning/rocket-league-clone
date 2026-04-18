#include "engine/physics/contact_constraint_solver.hpp"

#include <gtest/gtest.h>

#include "engine/physics/box.hpp"
#include "engine/physics/box_builder.hpp"
#include "engine/physics/collisions.hpp"
#include "engine/physics/contact.hpp"
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

TEST(BoxBoxSymmetry, OrientationIndependence) {
  // Baseline: head-on along Y axis
  Box a1 = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -1, 0).Id(1).Build();
  Box b1 = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();
  Contact c1;
  ASSERT_TRUE(Collisions::ComputeContact(a1, b1, c1));
  RunSolver(a1, b1, c1);

  // Same collision rotated by arbitrary quaternion
  glm::quat q = glm::normalize(glm::quat(0.5f, 0.3f, 0.7f, 0.1f));
  glm::vec3 pos_a = q * glm::vec3(0, 0.99f, 0);
  glm::vec3 pos_b = q * glm::vec3(0, -0.99f, 0);
  glm::vec3 vel_a = q * glm::vec3(0, -1, 0);
  glm::vec3 vel_b = q * glm::vec3(0, 1, 0);
  Box a2 = BoxBuilder().Size(2.0f).Position(pos_a).Mass(10).Velocity(vel_a).Rotation(q).Id(1).Build();
  Box b2 = BoxBuilder().Size(2.0f).Position(pos_b).Mass(10).Velocity(vel_b).Rotation(q).Id(2).Build();
  Contact c2;
  ASSERT_TRUE(Collisions::ComputeContact(a2, b2, c2));
  RunSolver(a2, b2, c2);

  float eps = 1e-3f;
  EXPECT_NEAR(glm::length(a1.velocity), glm::length(a2.velocity), eps);
  EXPECT_NEAR(glm::length(b1.velocity), glm::length(b2.velocity), eps);
  EXPECT_NEAR(glm::length(a1.angular_velocity), glm::length(a2.angular_velocity), eps);
  EXPECT_NEAR(glm::length(b1.angular_velocity), glm::length(b2.angular_velocity), eps);
}

TEST(BoxBoxSymmetry, ContactTypeEquivalence) {
  float eps = 0.1f;

  // Face-face: both aligned
  Box ff_a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -1, 0).Id(1).Build();
  Box ff_b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();
  Contact ff_c;
  ASSERT_TRUE(Collisions::ComputeContact(ff_a, ff_b, ff_c));
  RunSolver(ff_a, ff_b, ff_c);

  // Edge-face: rotate so edge midpoint direction (1,1,0) points down (-Y)
  glm::quat edge_rot = glm::rotation(glm::normalize(glm::vec3(1, 1, 0)), glm::vec3(0, -1, 0));
  Box ef_a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -1, 0).Rotation(edge_rot).Id(1).Build();
  Box ef_b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();
  Contact ef_c;
  ASSERT_TRUE(Collisions::ComputeContact(ef_a, ef_b, ef_c));
  RunSolver(ef_a, ef_b, ef_c);

  // Corner-face: rotate so corner direction (1,1,1) points down (-Y)
  glm::quat corner_rot = glm::rotation(glm::normalize(glm::vec3(1, 1, 1)), glm::vec3(0, -1, 0));
  Box cf_a =
      BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -1, 0).Rotation(corner_rot).Id(1).Build();
  Box cf_b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();
  Contact cf_c;
  ASSERT_TRUE(Collisions::ComputeContact(cf_a, cf_b, cf_c));
  RunSolver(cf_a, cf_b, cf_c);

  float ff_speed_a = glm::length(ff_a.velocity);
  float ef_speed_a = glm::length(ef_a.velocity);
  float cf_speed_a = glm::length(cf_a.velocity);
  EXPECT_NEAR(ff_speed_a, ef_speed_a, eps);
  EXPECT_NEAR(ff_speed_a, cf_speed_a, eps);
}

TEST(BoxBoxSymmetry, SwappedArguments) {
  // Run (a, b)
  Box a1 = BoxBuilder().Size(2.0f).Position(0.3f, 0.99f, 0).Mass(5).Velocity(0, -2, 0).Id(1).Build();
  Box b1 = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();
  Contact c1;
  ASSERT_TRUE(Collisions::ComputeContact(a1, b1, c1));
  RunSolver(a1, b1, c1);

  // Run (b, a) with fresh copies
  Box a2 = BoxBuilder().Size(2.0f).Position(0.3f, 0.99f, 0).Mass(5).Velocity(0, -2, 0).Id(1).Build();
  Box b2 = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();
  Contact c2;
  ASSERT_TRUE(Collisions::ComputeContact(b2, a2, c2));
  RunSolver(a2, b2, c2);

  float eps = 1e-4f;
  EXPECT_NEAR(a1.velocity.x, a2.velocity.x, eps);
  EXPECT_NEAR(a1.velocity.y, a2.velocity.y, eps);
  EXPECT_NEAR(a1.velocity.z, a2.velocity.z, eps);
  EXPECT_NEAR(b1.velocity.x, b2.velocity.x, eps);
  EXPECT_NEAR(b1.velocity.y, b2.velocity.y, eps);
  EXPECT_NEAR(b1.velocity.z, b2.velocity.z, eps);
  EXPECT_NEAR(glm::length(a1.angular_velocity), glm::length(a2.angular_velocity), eps);
  EXPECT_NEAR(glm::length(b1.angular_velocity), glm::length(b2.angular_velocity), eps);
}

TEST(BoxBoxRestitution, Elastic_ConservesKineticEnergy) {
  Box a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(5).Velocity(0, -2, 0).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  float ke_before = 0.5f * a.mass * glm::dot(a.velocity, a.velocity) +
                    0.5f * b.mass * glm::dot(b.velocity, b.velocity);

  RunSolver(a, b, contact, 1.0f);

  float ke_after = 0.5f * a.mass * glm::dot(a.velocity, a.velocity) +
                   0.5f * b.mass * glm::dot(b.velocity, b.velocity);

  EXPECT_NEAR(ke_before, ke_after, 1e-3f);
}

TEST(BoxBoxRestitution, PerfectlyInelastic_ZeroRelativeNormalVelocity) {
  Box a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(5).Velocity(0, -2, 0).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();
  glm::vec3 n(0, -1, 0);
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, n, {{midpoint, 0.02f}});

  RunSolver(a, b, contact, 0.0f);

  float v_rel_n = glm::dot(b.velocity - a.velocity, n);
  EXPECT_NEAR(v_rel_n, 0.0f, 1e-4f);
}

TEST(BoxBoxRestitution, Partial_MomentumConservedAndEnergyDecreases) {
  Box a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(5).Velocity(0, -2, 0).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  glm::vec3 momentum_before = a.mass * a.velocity + b.mass * b.velocity;
  float ke_before = 0.5f * a.mass * glm::dot(a.velocity, a.velocity) +
                    0.5f * b.mass * glm::dot(b.velocity, b.velocity);

  RunSolver(a, b, contact, 0.5f);

  glm::vec3 momentum_after = a.mass * a.velocity + b.mass * b.velocity;
  float ke_after = 0.5f * a.mass * glm::dot(a.velocity, a.velocity) +
                   0.5f * b.mass * glm::dot(b.velocity, b.velocity);

  EXPECT_NEAR(momentum_before.y, momentum_after.y, 1e-3f);
  EXPECT_LT(ke_after, ke_before);
}

TEST(BoxBoxPenetrationCorrection, ProducesSeparatingVelocity) {
  // Zero relative velocity, but penetrating — baumgarte should add separating velocity
  Box a = BoxBuilder().Size(2.0f).Position(0, 0.95f, 0).Mass(10).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.95f, 0).Mass(10).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  float penetration = 0.1f;
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, penetration}});

  RunSolver(a, b, contact, 0.0f, 0.2f);

  // Normal points from a to b (-Y). Baumgarte should push them apart:
  // a gains +Y velocity, b gains -Y velocity
  EXPECT_GT(a.velocity.y, 1e-4f);
  EXPECT_LT(b.velocity.y, -1e-4f);
}

TEST(BoxBoxPenetrationCorrection, ProportionalToInverseMass) {
  Box light = BoxBuilder().Size(2.0f).Position(0, 0.95f, 0).Mass(1).Id(1).Build();
  Box heavy = BoxBuilder().Size(2.0f).Position(0, -0.95f, 0).Mass(3).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  float penetration = 0.1f;
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, penetration}});

  RunSolver(light, heavy, contact, 0.0f, 0.2f);

  // Lighter body should receive more corrective velocity
  EXPECT_GT(std::abs(light.velocity.y), std::abs(heavy.velocity.y));
  // Ratio should be proportional to inverse mass ratio (3:1)
  float ratio = std::abs(light.velocity.y) / std::abs(heavy.velocity.y);
  EXPECT_NEAR(ratio, 3.0f, 0.1f);
}

TEST(BoxBoxPenetrationCorrection, DeeperPenetrationProducesLargerCorrection) {
  // Shallow penetration
  Box a1 = BoxBuilder().Size(2.0f).Position(0, 0.95f, 0).Mass(10).Id(1).Build();
  Box b1 = BoxBuilder().Size(2.0f).Position(0, -0.95f, 0).Mass(10).Id(2).Build();
  Contact c1 = MakeContact(1, 2, glm::vec3(0, -1, 0), {{glm::vec3(0), 0.05f}});
  RunSolver(a1, b1, c1, 0.0f, 0.2f);

  // Deep penetration
  Box a2 = BoxBuilder().Size(2.0f).Position(0, 0.95f, 0).Mass(10).Id(1).Build();
  Box b2 = BoxBuilder().Size(2.0f).Position(0, -0.95f, 0).Mass(10).Id(2).Build();
  Contact c2 = MakeContact(1, 2, glm::vec3(0, -1, 0), {{glm::vec3(0), 0.2f}});
  RunSolver(a2, b2, c2, 0.0f, 0.2f);

  EXPECT_GT(std::abs(a2.velocity.y), std::abs(a1.velocity.y));
  EXPECT_GT(std::abs(b2.velocity.y), std::abs(b1.velocity.y));
}

TEST(BoxBoxEdgeCase, LargeMassRatio) {
  Box light = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(1).Velocity(0, -1, 0).Id(1).Build();
  Box heavy = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(1000).Velocity(0, 0, 0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  RunSolver(light, heavy, contact);

  // Heavy box barely changes
  EXPECT_NEAR(heavy.velocity.y, 0.0f, 0.01f);
  // Light box is flung away (reflected)
  EXPECT_GT(light.velocity.y, 0.9f);
}

TEST(BoxBoxEdgeCase, GlancingCollision) {
  // Mostly tangential velocity with small normal component
  Box a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(5, -0.1f, 0).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  RunSolver(a, b, contact);

  // Tangential velocity (x) mostly retained — no friction in normal-only solver
  EXPECT_NEAR(a.velocity.x, 5.0f, 1e-4f);
  // Small normal impulse applied
  EXPECT_GT(a.velocity.y, -0.1f);
}

TEST(BoxBoxEdgeCase, MultipleContactPoints) {
  // Face-face generates multiple contact points; momentum should still be conserved
  Box a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(5).Velocity(0, -2, 0).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();

  Contact contact;
  ASSERT_TRUE(Collisions::ComputeContact(a, b, contact));
  ASSERT_GT(contact.points.size(), 1u);

  glm::vec3 momentum_before = a.mass * a.velocity + b.mass * b.velocity;

  RunSolver(a, b, contact);

  glm::vec3 momentum_after = a.mass * a.velocity + b.mass * b.velocity;
  float eps = 1e-3f;
  EXPECT_NEAR(momentum_before.x, momentum_after.x, eps);
  EXPECT_NEAR(momentum_before.y, momentum_after.y, eps);
  EXPECT_NEAR(momentum_before.z, momentum_after.z, eps);
}

TEST(BoxBoxImmovable, MovableBouncesOffImmovable) {
  Box movable = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -1, 0).Id(1).Build();
  Box immovable = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  RunSolver(movable, immovable, contact);

  // Movable bounces back
  EXPECT_NEAR(movable.velocity.y, 1.0f, 1e-4f);
  // Immovable unchanged
  EXPECT_NEAR(immovable.velocity.y, 0.0f, 1e-6f);
  EXPECT_NEAR(glm::length(immovable.angular_velocity), 0.0f, 1e-6f);
}

TEST(BoxBoxImmovable, PenetrationCorrection_OnlyMovableDisplaced) {
  Box movable = BoxBuilder().Size(2.0f).Position(0, 0.95f, 0).Mass(10).Id(1).Build();
  Box immovable = BoxBuilder().Size(2.0f).Position(0, -0.95f, 0).Mass(0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.1f}});

  RunSolver(movable, immovable, contact, 0.0f, 0.2f);

  // Only movable gets corrective velocity
  EXPECT_GT(movable.velocity.y, 1e-4f);
  // Immovable stays put
  EXPECT_NEAR(immovable.velocity.y, 0.0f, 1e-6f);
}

TEST(BoxBoxImmovable, KineticEnergyConserved_ElasticBounce) {
  Box movable = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -3, 0).Id(1).Build();
  Box immovable = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  float ke_before = 0.5f * movable.mass * glm::dot(movable.velocity, movable.velocity);

  RunSolver(movable, immovable, contact);

  float ke_after = 0.5f * movable.mass * glm::dot(movable.velocity, movable.velocity);
  EXPECT_NEAR(ke_before, ke_after, 1e-3f);
}

TEST(BoxBoxImmovable, Symmetry_ArgumentOrder) {
  // (movable, immovable)
  Box m1 = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -1, 0).Id(1).Build();
  Box i1 = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(0).Id(2).Build();
  Contact c1;
  ASSERT_TRUE(Collisions::ComputeContact(m1, i1, c1));
  RunSolver(m1, i1, c1);

  // (immovable, movable)
  Box m2 = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(10).Velocity(0, -1, 0).Id(1).Build();
  Box i2 = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(0).Id(2).Build();
  Contact c2;
  ASSERT_TRUE(Collisions::ComputeContact(i2, m2, c2));
  RunSolver(m2, i2, c2);

  float eps = 1e-4f;
  EXPECT_NEAR(m1.velocity.x, m2.velocity.x, eps);
  EXPECT_NEAR(m1.velocity.y, m2.velocity.y, eps);
  EXPECT_NEAR(m1.velocity.z, m2.velocity.z, eps);
}

TEST(BoxBoxImmovable, TwoImmovables_NoImpulse) {
  // Two immovable boxes: effective_mass is 0, so no impulse applied
  Box a = BoxBuilder().Size(2.0f).Position(0, 0.99f, 0).Mass(0).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -0.99f, 0).Mass(0).Id(2).Build();
  glm::vec3 midpoint(0, 0, 0);
  Contact contact = MakeContact(1, 2, glm::vec3(0, -1, 0), {{midpoint, 0.02f}});

  RunSolver(a, b, contact);

  EXPECT_NEAR(glm::length(a.velocity), 0.0f, 1e-6f);
  EXPECT_NEAR(glm::length(b.velocity), 0.0f, 1e-6f);
}

TEST(BoxBoxNoContact, NothingHappens) {
  Box a = BoxBuilder().Size(2.0f).Position(0, 5, 0).Mass(10).Velocity(0, -1, 0).Id(1).Build();
  Box b = BoxBuilder().Size(2.0f).Position(0, -5, 0).Mass(10).Velocity(0, 1, 0).Id(2).Build();

  Contact contact;
  ASSERT_FALSE(Collisions::ComputeContact(a, b, contact));

  // No contact means no constraints generated — velocities unchanged
  EXPECT_NEAR(a.velocity.y, -1.0f, 1e-6f);
  EXPECT_NEAR(b.velocity.y, 1.0f, 1e-6f);
}

}  // namespace engine::physics
