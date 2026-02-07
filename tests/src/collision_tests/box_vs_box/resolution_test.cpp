#include <box_builder.hpp>
#include <engine/physics/collisions.hpp>
#include <gtest/gtest.h>

TEST(BoxBoxResolution, OffCenterImpact_ProducesAngularVelocity) {
  glm::vec3 box_a_initial_velocity = glm::vec3();
  glm::vec3 box_b_initial_velocity = glm::vec3(-1.0f, 0.0f, 0.0f);
  auto box_a = BoxBuilder().Velocity(box_a_initial_velocity).Build();
  auto box_b = BoxBuilder()
                   .Position(0.999f, 0.5f, 0.0f)
                   .Velocity(box_b_initial_velocity)
                   .Build();
  engine::physics::Contact contact;

  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  EXPECT_TRUE(box_a.angular_velocity.z > box_a_initial_velocity.z)
      << "Box angular velocity did not increase in the appropriate direction "
         "after collision.";
}

// TODO - Test Cases
//
// --- Linear velocity / basic impulse ---
//    Head-on centered (Face-Face aligned) => no rotation
//      Two axis-aligned boxes collide face-to-face dead center. Only linear
//      velocities should change; angular velocity stays zero.
//    Asymmetric mass => lighter box gets more velocity
//      Heavy box hitting light box; verify the lighter one picks up more speed.
//    Same-direction catch-up => proper resolution
//      Faster box rear-ends a slower one along the same axis.
//
// --- Angular velocity ---
//    Face-Face misaligned => produces rotation
//      Boxes overlap on a face axis but one is offset in a tangent direction,
//      creating a torque.
//    Pure rotation, no translation => still produces impulse
//      One box is spinning in place and its corner/edge sweeps into a stationary
//      box. The contact-point velocity from cross(omega, r) should generate an
//      impulse.
//    Both boxes rotating => angular velocities change appropriately
//      Pre-existing angular velocity on both bodies; verify both are updated.
//
// --- Orientation independence ---
//    Non-identity rotation gives equivalent result
//      Rotate the entire scenario (both boxes) by some arbitrary quaternion. The
//      resulting velocity magnitudes should match the axis-aligned case.
//
// --- Contact-type equivalence ---
//    Edge-Face, Corner-Face, and Face-Face produce same linear impulse when
//    contact centroid is the same
//      Set up three scenarios where the contact point centroid is at the same
//      spot relative to both boxes' centers. The total linear impulse magnitude
//      should be equal (since r_a and r_b would be equivalent).
//
// --- Coefficient of restitution ---
//    e=1 (elastic) => kinetic energy conserved
//      Verify total KE (linear + rotational) is unchanged.
//    e=0 (perfectly inelastic) => relative velocity along normal is zero after
//      The two contact points should have the same normal-component velocity
//      post-resolution.
//    0 < e < 1 => energy decreases, momentum conserved
//      Verify momentum is conserved and KE is strictly less than before.
//
// --- Penetration correction ---
//    Boxes are fully separated after resolution
//      After ResolveCollision, the boxes should no longer overlap (re-run
//      ComputeContact and expect false).
//    Separation is proportional to inverse mass
//      A heavy box should move less than a light box during correction. Verify
//      the ratio matches m_b / m_a.
//
// --- Symmetry ---
//    Swapping box_a and box_b gives mirror-symmetric result
//      Swap the arguments; linear velocities should be mirrored, angular
//      velocities consistent.
//    Equal-mass head-on => velocities exchange
//      Classic elastic collision: two equal-mass boxes approaching each other
//      swap velocities.
//
// --- Edge cases ---
//    Very large mass ratio (e.g. 1000:1)
//      The heavy box should barely change velocity.
//    Glancing collision
//      Boxes barely clip a corner. The impulse should be small and mostly
//      tangential effects.
//    Multiple contact points (Face-Face) => consistent result
//      The impulse-per-point division (/ contact.points.size()) should still
//      produce physically correct totals.
//    Both boxes stationary but overlapping => penetration correction only
//      v_n is zero so impulse should be zero, but positions should still be
//      corrected apart.
