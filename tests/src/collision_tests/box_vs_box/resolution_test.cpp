#include <engine/physics/box_builder.hpp>
#include <engine/physics/collisions.hpp>
#include <gtest/gtest.h>
#include <print.hpp>
#include <stdexcept>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include "test_util.hpp"

TEST(BoxBoxResolution, HeadOnCollision_ProducesNoRotation) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  EXPECT_EQ(box_a.angular_velocity, glm::vec3()) << "Box A should have no rotation.";
  EXPECT_EQ(box_b.angular_velocity, glm::vec3()) << "Box B should have no rotation.";

  EXPECT_LT(box_a.velocity.x, 0.0f) << "Box A velocity should move towards the negative direction.";
  EXPECT_GT(box_b.velocity.x, -1.0f) << "Box B velocity should move towards the positive direction.";
}

TEST(BoxBoxResolution, LighterBoxMovesFaster) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Mass(3.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  EXPECT_EQ(box_a.velocity.x, -1.5f) << "Box A velocity should move towards the negative direction quickly.";
  EXPECT_EQ(box_b.velocity.x, -0.5f) << "Box B velocity should move towards the positive direction slowly.";
}

TEST(BoxBoxResolution, SameDirectionCollision) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder()
    .Velocity(glm::vec3(-0.5f, 0.0f, 0.0f))
    .Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Mass(3.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  EXPECT_LT(box_a.velocity.x, -0.5f) << "Box A should speed up.";
  EXPECT_GT(box_b.velocity.x, -1.0f) << "Box B should slow down.";
}

TEST(BoxBoxResolution, ZeroRelativeVelocity_ShouldCreateNoImpulse) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder()
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Mass(3.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  EXPECT_EQ(box_a.velocity, glm::vec3(-1.0f, 0.0f, 0.0f))
      << "Box A velocity is equal to box B and should not cause an impulse.";
  EXPECT_EQ(box_b.velocity, glm::vec3(-1.0f, 0.0f, 0.0f))
      << "Box B velocity is equal to box A and should not cause an impulse.";
}

TEST(BoxBoxResolution, OffCenterImpact_ProducesAngularVelocity) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  EXPECT_GT(box_a.angular_velocity.z, 0) << "Box A should now be rotating counter-clockwise.";
  EXPECT_GT(box_b.angular_velocity.z, 0) << "Box B should now be rotating counter-clockwise.";

  EXPECT_LT(box_a.velocity.x, 0.0f) << "Box A velocity should move towards the negative direction.";
  EXPECT_GT(box_b.velocity.x, -1.0f) << "Box B velocity should move towards the positive direction.";
}

TEST(BoxBoxResolution, RotationOnlyCollision) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .AngularVelocity(0.0f, 0.0f, -5.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  EXPECT_GT(box_b.angular_velocity.z, -5.0f) << "Box B should lose some rotation.";
  EXPECT_LT(box_b.angular_velocity.z, 0.0f) << "Box B should still retain some rotation.";
  EXPECT_GT(box_a.angular_velocity.z, 0.0f) << "Box A should gain some rotation.";
  EXPECT_LT(box_a.velocity.x, 0.0f) << "Box A should gain some velocity.";
}

TEST(BoxBoxResolution, OrientationIndependence) {
  // --- Axis-aligned baseline (same as OffCenterImpact) ---
  // clang-format off
  auto box_a_aa = engine::physics::BoxBuilder().Build();
  auto box_b_aa = engine::physics::BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a_aa, box_b_aa);

  // --- Same scenario rotated by an arbitrary quaternion ---
  glm::quat q = glm::angleAxis(glm::radians(47.0f), glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f)));

  // clang-format off
  auto box_a_rot = engine::physics::BoxBuilder()
    .Rotation(q)
    .Build();
  auto box_b_rot = engine::physics::BoxBuilder()
    .Position(q * glm::vec3(0.999f, 0.5f, 0.0f))
    .Velocity(q * glm::vec3(-1.0f, 0.0f, 0.0f))
    .Rotation(q)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a_rot, box_b_rot);

  // Velocity magnitudes should match the axis-aligned case
  float tol = 1e-3f;
  EXPECT_NEAR(glm::length(box_a_rot.velocity), glm::length(box_a_aa.velocity), tol)
      << "Box A linear speed should match the axis-aligned case.";
  EXPECT_NEAR(glm::length(box_b_rot.velocity), glm::length(box_b_aa.velocity), tol)
      << "Box B linear speed should match the axis-aligned case.";
  EXPECT_NEAR(glm::length(box_a_rot.angular_velocity), glm::length(box_a_aa.angular_velocity), tol)
      << "Box A angular speed should match the axis-aligned case.";
  EXPECT_NEAR(glm::length(box_b_rot.angular_velocity), glm::length(box_b_aa.angular_velocity), tol)
      << "Box B angular speed should match the axis-aligned case.";
}

TEST(BoxBoxResolution, ContactTypeEquivalence) {
  // Face-Face: both axis-aligned, incident box has face parallel to penetration axis
  // clang-format off
  auto ff_a = engine::physics::BoxBuilder().Build();
  auto ff_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(ff_a, ff_b);

  // Edge-Face: box_b rotated 45° around z so its edge leads into box_a's face.
  // Incident z-axis is perpendicular to penetration axis, no axis parallel.
  glm::quat edge_rot = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  // clang-format off
  auto ef_a = engine::physics::BoxBuilder().Build();
  auto ef_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Rotation(edge_rot)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(ef_a, ef_b);

  // Corner-Face: box_b rotated so no axis is parallel or perpendicular to
  // penetration axis, triggering the ClipCornerToFace path.
  glm::quat corner_rot = glm::angleAxis(glm::radians(45.0f), glm::normalize(glm::vec3(0.0f, 1.0f, 1.0f)));
  // clang-format off
  auto cf_a = engine::physics::BoxBuilder().Build();
  auto cf_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Rotation(corner_rot)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(cf_a, cf_b);

  // For symmetric head-on collisions, Face-Face and Edge-Face both produce
  // centroids on the center line (r parallel to normal), so their angular
  // effective-mass terms vanish and impulses are identical.
  // Corner-Face has an off-axis contact point introducing a small angular
  // term, so we allow a wider tolerance.
  float ff_speed = glm::length(ff_a.velocity);
  float ef_speed = glm::length(ef_a.velocity);
  float cf_speed = glm::length(cf_a.velocity);

  EXPECT_NEAR(ef_speed, ff_speed, 1e-3f) << "Edge-Face and Face-Face should produce the same linear impulse.";
  EXPECT_NEAR(cf_speed, ff_speed, 0.15f) << "Corner-Face should produce a similar linear impulse to Face-Face.";
}

namespace {
float TotalKE(const engine::physics::Box& a, const engine::physics::Box& b) {
  auto ke = [](const engine::physics::Box& box) {
    float linear = 0.5f * box.mass * glm::dot(box.velocity, box.velocity);
    glm::mat3 rot = glm::toMat3(box.rotation);
    glm::mat3 i_world = rot * box.inertia_tensor * glm::transpose(rot);
    float rotational = 0.5f * glm::dot(box.angular_velocity, i_world * box.angular_velocity);
    return linear + rotational;
  };
  return ke(a) + ke(b);
}
}  // namespace

TEST(BoxBoxResolution, CoefficientOfRestitution_Elastic) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  float ke_before = TotalKE(box_a, box_b);

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  float ke_after = TotalKE(box_a, box_b);
  EXPECT_NEAR(ke_after, ke_before, 1e-3f) << "e=1: total kinetic energy should be conserved.";
}

TEST(BoxBoxResolution, CoefficientOfRestitution_PerfectlyInelastic) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on

  // Use ComputeContact to get centroid/normal for post-resolution verification
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));

  engine::physics::Collisions::HandleCollision(box_a, box_b, 0.0f);

  // Compute relative contact-point velocity along the normal after resolution
  glm::vec3 centroid(0.0f);
  for (const auto& p : contact.points) centroid += p;
  centroid /= static_cast<float>(contact.points.size());

  glm::vec3 r_a = centroid - box_a.position;
  glm::vec3 r_b = centroid - box_b.position;
  glm::vec3 v_contact_a = box_a.velocity + glm::cross(box_a.angular_velocity, r_a);
  glm::vec3 v_contact_b = box_b.velocity + glm::cross(box_b.angular_velocity, r_b);
  float v_rel_normal = glm::dot(v_contact_a - v_contact_b, contact.normal);

  EXPECT_NEAR(v_rel_normal, 0.0f, 1e-3f) << "e=0: relative contact-point velocity along normal should be zero.";
}

TEST(BoxBoxResolution, CoefficientOfRestitution_Partial) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  glm::vec3 p_before = box_a.mass * box_a.velocity + box_b.mass * box_b.velocity;
  float ke_before = TotalKE(box_a, box_b);

  engine::physics::Collisions::HandleCollision(box_a, box_b, 0.5f);

  glm::vec3 p_after = box_a.mass * box_a.velocity + box_b.mass * box_b.velocity;
  float ke_after = TotalKE(box_a, box_b);

  EXPECT_NEAR(p_after.x, p_before.x, 1e-3f) << "e=0.5: linear momentum (x) should be conserved.";
  EXPECT_NEAR(p_after.y, p_before.y, 1e-3f) << "e=0.5: linear momentum (y) should be conserved.";
  EXPECT_NEAR(p_after.z, p_before.z, 1e-3f) << "e=0.5: linear momentum (z) should be conserved.";
  EXPECT_LT(ke_after, ke_before) << "e=0.5: total kinetic energy should decrease.";
}

TEST(BoxBoxResolution, PenetrationCorrection_FullySeparated) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  engine::physics::Contact post_contact;
  EXPECT_FALSE(engine::physics::Collisions::ComputeContact(box_a, box_b, post_contact))
      << "Boxes should no longer overlap after penetration correction.";
}

TEST(BoxBoxResolution, PenetrationCorrection_ProportionalToInverseMass) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder()
    .Mass(1.0f)
    .Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Mass(3.0f)
    .Build();
  // clang-format on
  glm::vec3 pos_a_before = box_a.position;
  glm::vec3 pos_b_before = box_b.position;

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  float disp_a = glm::length(box_a.position - pos_a_before);
  float disp_b = glm::length(box_b.position - pos_b_before);

  // displacement_a / displacement_b should equal mass_b / mass_a
  EXPECT_NEAR(disp_a / disp_b, box_b.mass / box_a.mass, 1e-3f)
      << "Lighter box should be displaced more, proportional to inverse mass.";
}

TEST(BoxBoxResolution, Symmetry_SwappedArguments) {
  // Run 1: HandleCollision(a, b)
  // clang-format off
  auto a1 = engine::physics::BoxBuilder().Build();
  auto b1 = engine::physics::BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(a1, b1);

  // Run 2: same setup but swapped argument order
  // clang-format off
  auto a2 = engine::physics::BoxBuilder().Build();
  auto b2 = engine::physics::BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(b2, a2);

  float tol = 1e-3f;
  EXPECT_NEAR(a2.velocity.x, a1.velocity.x, tol);
  EXPECT_NEAR(a2.velocity.y, a1.velocity.y, tol);
  EXPECT_NEAR(a2.velocity.z, a1.velocity.z, tol);
  EXPECT_NEAR(b2.velocity.x, b1.velocity.x, tol);
  EXPECT_NEAR(b2.velocity.y, b1.velocity.y, tol);
  EXPECT_NEAR(b2.velocity.z, b1.velocity.z, tol);
  EXPECT_NEAR(glm::length(a2.angular_velocity), glm::length(a1.angular_velocity), tol);
  EXPECT_NEAR(glm::length(b2.angular_velocity), glm::length(b1.angular_velocity), tol);
}

TEST(BoxBoxResolution, Symmetry_EqualMassHeadOn_VelocitiesExchange) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder()
    .Velocity(1.0f, 0.0f, 0.0f)
    .Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  float tol = 1e-3f;
  EXPECT_NEAR(box_a.velocity.x, -1.0f, tol) << "Box A should have box B's original velocity.";
  EXPECT_NEAR(box_b.velocity.x, 1.0f, tol) << "Box B should have box A's original velocity.";
}

TEST(BoxBoxResolution, EdgeCase_LargeMassRatio) {
  // clang-format off
  auto light = engine::physics::BoxBuilder().Build();
  auto heavy = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Mass(1000.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(light, heavy);

  EXPECT_NEAR(heavy.velocity.x, -1.0f, 0.01f) << "Heavy box should barely change velocity.";
  EXPECT_LT(light.velocity.x, -1.0f) << "Light box should be moving fast in the negative direction.";
}

TEST(BoxBoxResolution, EdgeCase_GlancingCollision) {
  // Box B approaches mostly tangentially with a small normal component
  // clang-format off
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-0.1f, -1.0f, 0.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  // The impulse should be small since the normal-component of relative velocity is small
  float speed_change_a = glm::length(box_a.velocity);
  EXPECT_LT(speed_change_a, 0.5f) << "Glancing collision should produce a small impulse.";
  // Box B should retain most of its tangential velocity
  EXPECT_LT(box_b.velocity.y, -0.5f) << "Box B should retain most of its tangential velocity.";
}

TEST(BoxBoxResolution, EdgeCase_MultipleContactPoints) {
  // Face-Face collision produces multiple contact points.
  // Verify momentum is conserved even with multi-point contacts.
  // clang-format off
  auto box_a = engine::physics::BoxBuilder().Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on

  // Use ComputeContact to verify multiple contact points are generated
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  ASSERT_GT(contact.points.size(), 1u) << "Face-Face should produce multiple contact points.";

  glm::vec3 p_before = box_a.mass * box_a.velocity + box_b.mass * box_b.velocity;
  engine::physics::Collisions::HandleCollision(box_a, box_b);
  glm::vec3 p_after = box_a.mass * box_a.velocity + box_b.mass * box_b.velocity;

  float tol = 1e-3f;
  EXPECT_NEAR(p_after.x, p_before.x, tol) << "Momentum (x) should be conserved with multiple contact points.";
  EXPECT_NEAR(p_after.y, p_before.y, tol) << "Momentum (y) should be conserved with multiple contact points.";
  EXPECT_NEAR(p_after.z, p_before.z, tol) << "Momentum (z) should be conserved with multiple contact points.";
}

// --- Immovable box (mass <= 0) tests ---

TEST(BoxBoxImmovable, MovableBouncesOffImmovable) {
  // clang-format off
  auto movable = engine::physics::BoxBuilder()
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  auto immovable = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Mass(0.0f)
    .Build();
  // clang-format on
  glm::vec3 immovable_vel_before = immovable.velocity;
  glm::vec3 immovable_pos_before = immovable.position;

  engine::physics::Collisions::HandleCollision(movable, immovable);

  EXPECT_GT(movable.velocity.x, 0.0f) << "Movable box should bounce back.";
  TestUtil::ExpectVec3Near(immovable_vel_before, immovable.velocity, "Immovable box velocity should not change");
  TestUtil::ExpectVec3Near(immovable_pos_before, immovable.position, "Immovable box position should not change");
  EXPECT_EQ(immovable.angular_velocity, glm::vec3(0.0f)) << "Immovable box should not rotate.";
}

TEST(BoxBoxImmovable, PenetrationCorrection_OnlyMovableDisplaced) {
  // clang-format off
  auto movable = engine::physics::BoxBuilder()
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  auto immovable = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Mass(0.0f)
    .Build();
  // clang-format on
  glm::vec3 immovable_pos_before = immovable.position;

  engine::physics::Collisions::HandleCollision(movable, immovable);

  TestUtil::ExpectVec3Near(immovable_pos_before, immovable.position, "Immovable box should not be displaced");

  engine::physics::Contact post_contact;
  EXPECT_FALSE(engine::physics::Collisions::ComputeContact(movable, immovable, post_contact))
      << "Boxes should no longer overlap after penetration correction.";
}

TEST(BoxBoxImmovable, KineticEnergyConserved_ElasticBounce) {
  // clang-format off
  auto movable = engine::physics::BoxBuilder()
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  auto immovable = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Mass(0.0f)
    .Build();
  // clang-format on

  float ke_before = TotalKE(movable, immovable);

  engine::physics::Collisions::HandleCollision(movable, immovable);

  float ke_after = TotalKE(movable, immovable);

  EXPECT_NEAR(ke_after, ke_before, 1e-3f) << "Kinetic energy should be conserved bouncing off immovable box.";
}

TEST(BoxBoxImmovable, Symmetry_ArgumentOrder) {
  // clang-format off
  auto movable1 = engine::physics::BoxBuilder()
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  auto immovable1 = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Mass(0.0f)
    .Build();
  auto movable2 = engine::physics::BoxBuilder()
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  auto immovable2 = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Mass(0.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(movable1, immovable1);
  engine::physics::Collisions::HandleCollision(immovable2, movable2);

  float tol = 1e-3f;
  EXPECT_NEAR(movable1.velocity.x, movable2.velocity.x, tol);
  EXPECT_NEAR(movable1.velocity.y, movable2.velocity.y, tol);
  EXPECT_NEAR(movable1.velocity.z, movable2.velocity.z, tol);
  TestUtil::ExpectVec3Near(immovable1.velocity, immovable2.velocity,
                           "Immovable velocity should match regardless of arg order");
  TestUtil::ExpectVec3Near(immovable1.position, immovable2.position,
                           "Immovable position should match regardless of arg order");
}

TEST(BoxBoxImmovable, TwoImmovables_ShouldThrowError) {
  // clang-format off
  auto immovable_a = engine::physics::BoxBuilder()
    .Mass(0.0f)
    .Build();
  auto immovable_b = engine::physics::BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Mass(0.0f)
    .Build();
  // clang-format on
  EXPECT_THROW(engine::physics::Collisions::HandleCollision(immovable_a, immovable_b), std::logic_error)
      << "Resolving collision between two immovable objects should throw an error.";
}

// --- HandleCollision: no-contact early return ---

TEST(HandleCollision, NoContact_NothingHappens) {
  // clang-format off
  auto box_a = engine::physics::BoxBuilder()
    .Velocity(1.0f, 0.0f, 0.0f)
    .Build();
  auto box_b = engine::physics::BoxBuilder()
    .Position(5.0f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on

  engine::physics::Collisions::HandleCollision(box_a, box_b);

  EXPECT_EQ(box_a.velocity, glm::vec3(1.0f, 0.0f, 0.0f)) << "No contact, velocity unchanged.";
  EXPECT_EQ(box_b.velocity, glm::vec3(-1.0f, 0.0f, 0.0f)) << "No contact, velocity unchanged.";
}
