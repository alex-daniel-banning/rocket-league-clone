#include <Print.hpp>
#include <engine/match.hpp>
#include <engine/physics/Contact.hpp>
#include <engine/physics/box.hpp>
#include <engine/physics/collisions.hpp>

namespace engine {

void Match::Tick(float delta_time) {
  accumulator_ += delta_time;
  while (accumulator_ >= fixed_dt) {
    // Define how much damping PER SECOND
    const float linear_damping_per_sec = 0.98f;
    const float angular_damping_per_sec = 0.95f;

    // Convert to per-frame damping
    float linear_damp = std::pow(linear_damping_per_sec, fixed_dt);
    float angular_damp = std::pow(angular_damping_per_sec, fixed_dt);

    ball_.position += fixed_dt * ball_.velocity;
    // damping
    ball_.velocity *= linear_damp;
    for (physics::Box& box : boxes_) {
      box.position += fixed_dt * box.velocity;
      glm::quat q = box.rotation;
      glm::vec3 w = box.angular_velocity;
      box.rotation = glm::normalize(
          q + (0.5f * fixed_dt * q * glm::quat(0, w.x, w.y, w.z)));
      box.velocity *= linear_damp;
      box.angular_velocity *= angular_damp;
    }
    HandleCollisions();
    accumulator_ -= fixed_dt;
  }
}

void Match::Reset() {
  ball_ = initial_ball_;
  boxes_ = initial_boxes_;
}

void Match::HandleCollisions() {
  for (physics::Plane wall : walls_) {
    physics::Collisions::HandleElasticCollision(wall, ball_);
  }
  physics::Collisions::HandleElasticCollision(ground_, ball_);

  for (physics::Box& box : boxes_) {
    physics::Contact contact;
    if (physics::Collisions::ComputeContact(box, ball_, contact)) {
      physics::Collisions::ResolveElasticCollision(box, ball_, contact);
    }
  }
}

}  // namespace engine
