#include <engine/physics/collisions.hpp>
#include <gtest/gtest.h>

TEST(BoxBoxResolution, OffCenterImpact_ProducesAngularVelocity) {
  glm::vec3 box_a_initial_velocity = glm::vec3();
  glm::vec3 box_b_initial_velocity = glm::vec3(-1.0f, 0.0f, 0.0f);
  engine::physics::Box box_a(glm::vec3(1.0f), glm::vec3(0.0f), box_a_initial_velocity, 1.0f, glm::quat());
  engine::physics::Box box_b(glm::vec3(1.0f), glm::vec3(0.999f, 0.5f, 0.0f), box_b_initial_velocity, 1.0f, glm::quat());
  engine::physics::Contact contact;

  ASSERT_TRUE(engine::physics::Collisions::ComputeContact(box_a, box_b, contact));
  engine::physics::Collisions::ResolveCollision(box_a, box_b, contact, 1.0f);

  EXPECT_TRUE(box_a.angular_velocity.z > box_a_initial_velocity.z)
      << "Box angular velocity did not increase in the appropriate direction "
         "after collision.";
}

// TODO - Test Cases
//    Face-Face aligned => no rotation
//    Face-Face misaligend => rotation
//    Face-Face is agnostic of orientation (non-identity box)
//    Edge-Face, Corner-Face, and Face-Face all produce same level of impact when center of impact matches
//    Rotation with no translation velocity still causes impulse
//    ...
