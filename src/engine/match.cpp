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

    if (ball_) {
      ball_->position += fixed_dt * ball_->velocity;
      ball_->velocity *= linear_damp;
    }
    for (physics::Box& box : boxes_) {
      box.position += fixed_dt * box.velocity;
      glm::quat q = box.rotation;
      glm::vec3 w = box.angular_velocity;
      box.rotation = glm::normalize(q + (0.5f * fixed_dt * q * glm::quat(0, w.x, w.y, w.z)));
      box.velocity *= linear_damp;
      box.angular_velocity *= angular_damp;
    }
    HandleCollisions();
    accumulator_ -= fixed_dt;
  }
}

void Match::Reset() {
  ball_ = initial_ball_;
  boxes_ = {initial_boxes_.begin(), initial_boxes_.end()};
}

void Match::HandleCollisions() {
  if (ball_) {
    // Ball v Wall collisions
    for (physics::Box wall : walls_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(wall, *ball_, contact)) {
        physics::Collisions::ResolveElasticCollision(wall, *ball_, contact);
      }
    }
    // Ball v Ground collisions
    if (ground_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(*ground_, *ball_, contact)) {
        physics::Collisions::ResolveElasticCollision(*ground_, *ball_, contact);
      }
    }
    // Ball v Car collisions
    for (physics::Box& car : boxes_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(car, *ball_, contact)) {
        physics::Collisions::ResolveElasticCollision(car, *ball_, contact);
      }
    }
  }

  // Car v Car collisions
  for (unsigned int i = 0; i < boxes_.size() - 1; i++) {
    for (unsigned int j = i + 1; j < boxes_.size(); j++) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(boxes_[i], boxes_[j], contact)) {
        physics::Collisions::ResolveCollision(boxes_[i], boxes_[j], contact, 1.0f);
      }
    }
  }
  // Car v Wall collisions
  for (physics::Box& car : boxes_) {
    for (physics::Box& wall : walls_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(car, wall, contact)) {
        physics::Collisions::ResolveCollision(car, wall, contact, 1.0f);
      }
    }
  }
  // Car v Ground collisions
  for (physics::Box& car : boxes_) {
    physics::Contact contact;
    if (physics::Collisions::ComputeContact(car, *ground_, contact)) {
      physics::Collisions::ResolveCollision(car, *ground_, contact, 1.0f);
    }
  }
}

}  // namespace engine
