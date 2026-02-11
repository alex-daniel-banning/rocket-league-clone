#include <Print.hpp>
#include <engine/physics/box_builder.hpp>
#include <engine/physics/collisions.hpp>
#include <gtest/gtest.h>

TEST(BoxBoxResolution, HeadOnCollision_ProducesNoRotation) {
  // clang-format off
  auto box_a = BoxBuilder().Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Build();
  // clang-format on
  engine::physics::Contact contact;

  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  EXPECT_EQ(box_a.angular_velocity, glm::vec3()) << "Box A should have no rotation.";
  EXPECT_EQ(box_b.angular_velocity, glm::vec3()) << "Box B should have no rotation.";

  EXPECT_LT(box_a.velocity.x, 0.0f) << "Box A velocity should move towards the negative direction.";
  EXPECT_GT(box_b.velocity.x, -1.0f) << "Box B velocity should move towards the positive direction.";
}

TEST(BoxBoxResolution, LighterBoxMovesFaster) {
  // clang-format off
  auto box_a = BoxBuilder().Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Mass(3.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;

  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  EXPECT_EQ(box_a.velocity.x, -1.5f) << "Box A velocity should move towards the negative direction quickly.";
  EXPECT_EQ(box_b.velocity.x, -0.5f) << "Box B velocity should move towards the positive direction slowly.";
}

TEST(BoxBoxResolution, SameDirectionCollision) {
  // clang-format off
  auto box_a = BoxBuilder()
    .Velocity(glm::vec3(-0.5f, 0.0f, 0.0f))
    .Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Mass(3.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;

  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  EXPECT_LT(box_a.velocity.x, -0.5f) << "Box A should speed up.";
  EXPECT_GT(box_b.velocity.x, -1.0f) << "Box B should slow down.";
}

TEST(BoxBoxResolution, ZeroRelativeVelocity_ShouldCreateNoImpulse) {
  // clang-format off
  auto box_a = BoxBuilder()
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Mass(3.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;

  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  EXPECT_EQ(box_a.velocity, glm::vec3(-1.0f, 0.0f, 0.0f))
      << "Box A velocity is equal to box B and should not cause an impulse.";
  EXPECT_EQ(box_b.velocity, glm::vec3(-1.0f, 0.0f, 0.0f))
      << "Box B velocity is equal to box A and should not cause an impulse.";
}

TEST(BoxBoxResolution, OffCenterImpact_ProducesAngularVelocity) {
  // clang-format off
  auto box_a = BoxBuilder().Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(glm::vec3(-1.0f, 0.0f, 0.0f))
    .Build();
  // clang-format on
  engine::physics::Contact contact;

  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  EXPECT_GT(box_a.angular_velocity.z, 0) << "Box A should now be rotating counter-clockwise.";
  EXPECT_GT(box_b.angular_velocity.z, 0) << "Box B should now be rotating counter-clockwise.";

  EXPECT_LT(box_a.velocity.x, 0.0f) << "Box A velocity should move towards the negative direction.";
  EXPECT_GT(box_b.velocity.x, -1.0f) << "Box B velocity should move towards the positive direction.";
}

TEST(BoxBoxResolution, RotationOnlyCollision) {
  // clang-format off
  auto box_a = BoxBuilder().Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .AngularVelocity(0.0f, 0.0f, -5.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;

  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  EXPECT_GT(box_b.angular_velocity.z, -5.0f) << "Box B should lose some rotation.";
  EXPECT_LT(box_b.angular_velocity.z, 0.0f) << "Box B should still retain some rotation.";
  EXPECT_GT(box_a.angular_velocity.z, 0.0f) << "Box A should gain some rotation.";
  EXPECT_LT(box_a.velocity.x, 0.0f) << "Box A should gain some velocity.";
}

TEST(BoxBoxResolution, OrientationIndependence) {
  // --- Axis-aligned baseline (same as OffCenterImpact) ---
  // clang-format off
  auto box_a_aa = BoxBuilder().Build();
  auto box_b_aa = BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact_aa;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a_aa, box_b_aa, contact_aa));
  engine::physics::Collisions::ResolveCollision(box_a_aa, box_b_aa, contact_aa, 1.0f);

  // --- Same scenario rotated by an arbitrary quaternion ---
  glm::quat q = glm::angleAxis(glm::radians(47.0f), glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f)));

  // clang-format off
  auto box_a_rot = BoxBuilder()
    .Rotation(q)
    .Build();
  auto box_b_rot = BoxBuilder()
    .Position(q * glm::vec3(0.999f, 0.5f, 0.0f))
    .Velocity(q * glm::vec3(-1.0f, 0.0f, 0.0f))
    .Rotation(q)
    .Build();
  // clang-format on
  engine::physics::Contact contact_rot;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a_rot, box_b_rot, contact_rot));
  engine::physics::Collisions::ResolveCollision(box_a_rot, box_b_rot, contact_rot, 1.0f);

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
  auto ff_a = BoxBuilder().Build();
  auto ff_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  engine::physics::Contact ff_contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(ff_a, ff_b, ff_contact));
  engine::physics::Collisions::ResolveCollision(ff_a, ff_b, ff_contact, 1.0f);

  // Edge-Face: box_b rotated 45° around z so its edge leads into box_a's face.
  // Incident z-axis is perpendicular to penetration axis, no axis parallel.
  glm::quat edge_rot = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  // clang-format off
  auto ef_a = BoxBuilder().Build();
  auto ef_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Rotation(edge_rot)
    .Build();
  // clang-format on
  engine::physics::Contact ef_contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(ef_a, ef_b, ef_contact));
  engine::physics::Collisions::ResolveCollision(ef_a, ef_b, ef_contact, 1.0f);

  // Corner-Face: box_b rotated so no axis is parallel or perpendicular to
  // penetration axis, triggering the ClipCornerToFace path.
  glm::quat corner_rot = glm::angleAxis(glm::radians(45.0f), glm::normalize(glm::vec3(0.0f, 1.0f, 1.0f)));
  // clang-format off
  auto cf_a = BoxBuilder().Build();
  auto cf_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Rotation(corner_rot)
    .Build();
  // clang-format on
  engine::physics::Contact cf_contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(cf_a, cf_b, cf_contact));
  engine::physics::Collisions::ResolveCollision(cf_a, cf_b, cf_contact, 1.0f);

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
  auto box_a = BoxBuilder().Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;
  float ke_before = TotalKE(box_a, box_b);

  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  float ke_after = TotalKE(box_a, box_b);
  EXPECT_NEAR(ke_after, ke_before, 1e-3f) << "e=1: total kinetic energy should be conserved.";
}

TEST(BoxBoxResolution, CoefficientOfRestitution_PerfectlyInelastic) {
  // clang-format off
  auto box_a = BoxBuilder().Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;

  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 0.0f);

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
  auto box_a = BoxBuilder().Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  glm::vec3 p_before = box_a.mass * box_a.velocity + box_b.mass * box_b.velocity;
  float ke_before = TotalKE(box_a, box_b);

  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 0.5f);

  glm::vec3 p_after = box_a.mass * box_a.velocity + box_b.mass * box_b.velocity;
  float ke_after = TotalKE(box_a, box_b);

  EXPECT_NEAR(p_after.x, p_before.x, 1e-3f) << "e=0.5: linear momentum (x) should be conserved.";
  EXPECT_NEAR(p_after.y, p_before.y, 1e-3f) << "e=0.5: linear momentum (y) should be conserved.";
  EXPECT_NEAR(p_after.z, p_before.z, 1e-3f) << "e=0.5: linear momentum (z) should be conserved.";
  EXPECT_LT(ke_after, ke_before) << "e=0.5: total kinetic energy should decrease.";
}

TEST(BoxBoxResolution, PenetrationCorrection_FullySeparated) {
  // clang-format off
  auto box_a = BoxBuilder().Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  engine::physics::Contact post_contact;
  EXPECT_FALSE(engine::physics::Collisions::ComputeContact(box_a, box_b, post_contact))
      << "Boxes should no longer overlap after penetration correction.";
}

TEST(BoxBoxResolution, PenetrationCorrection_ProportionalToInverseMass) {
  // clang-format off
  auto box_a = BoxBuilder()
    .Mass(1.0f)
    .Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Mass(3.0f)
    .Build();
  // clang-format on
  glm::vec3 pos_a_before = box_a.position;
  glm::vec3 pos_b_before = box_b.position;

  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  float disp_a = glm::length(box_a.position - pos_a_before);
  float disp_b = glm::length(box_b.position - pos_b_before);

  // displacement_a / displacement_b should equal mass_b / mass_a
  EXPECT_NEAR(disp_a / disp_b, box_b.mass / box_a.mass, 1e-3f)
      << "Lighter box should be displaced more, proportional to inverse mass.";
}

TEST(BoxBoxResolution, Symmetry_SwappedArguments) {
  // Run 1: ComputeContact(a, b), ResolveCollision(a, b)
  // clang-format off
  auto a1 = BoxBuilder().Build();
  auto b1 = BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact1;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(a1, b1, contact1));
  engine::physics::Collisions::ResolveCollision(a1, b1, contact1, 1.0f);

  // Run 2: same setup but swapped argument order
  // clang-format off
  auto a2 = BoxBuilder().Build();
  auto b2 = BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact2;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(b2, a2, contact2));
  engine::physics::Collisions::ResolveCollision(b2, a2, contact2, 1.0f);

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
  auto box_a = BoxBuilder()
    .Velocity(1.0f, 0.0f, 0.0f)
    .Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  float tol = 1e-3f;
  EXPECT_NEAR(box_a.velocity.x, -1.0f, tol) << "Box A should have box B's original velocity.";
  EXPECT_NEAR(box_b.velocity.x, 1.0f, tol) << "Box B should have box A's original velocity.";
}

TEST(BoxBoxResolution, EdgeCase_LargeMassRatio) {
  // clang-format off
  auto light = BoxBuilder().Build();
  auto heavy = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Mass(1000.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(light, heavy, contact));
  engine::physics::Collisions::ResolveCollision(light, heavy, contact, 1.0f);

  EXPECT_NEAR(heavy.velocity.x, -1.0f, 0.01f) << "Heavy box should barely change velocity.";
  EXPECT_LT(light.velocity.x, -1.0f) << "Light box should be moving fast in the negative direction.";
}

TEST(BoxBoxResolution, EdgeCase_GlancingCollision) {
  // Box B approaches mostly tangentially with a small normal component
  // clang-format off
  auto box_a = BoxBuilder().Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.5f, 0.0f)
    .Velocity(-0.1f, -1.0f, 0.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

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
  auto box_a = BoxBuilder().Build();
  auto box_b = BoxBuilder()
    .Position(0.999f, 0.0f, 0.0f)
    .Velocity(-1.0f, 0.0f, 0.0f)
    .Build();
  // clang-format on
  engine::physics::Contact contact;
  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  ASSERT_GT(contact.points.size(), 1u) << "Face-Face should produce multiple contact points.";

  glm::vec3 p_before = box_a.mass * box_a.velocity + box_b.mass * box_b.velocity;
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);
  glm::vec3 p_after = box_a.mass * box_a.velocity + box_b.mass * box_b.velocity;

  float tol = 1e-3f;
  EXPECT_NEAR(p_after.x, p_before.x, tol) << "Momentum (x) should be conserved with multiple contact points.";
  EXPECT_NEAR(p_after.y, p_before.y, tol) << "Momentum (y) should be conserved with multiple contact points.";
  EXPECT_NEAR(p_after.z, p_before.z, tol) << "Momentum (z) should be conserved with multiple contact points.";
}
