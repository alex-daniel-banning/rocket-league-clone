#include "engine/match.hpp"

#include "engine/log.hpp"
#include "engine/physics/box.hpp"
#include "engine/physics/collisions.hpp"
#include "engine/physics/contact.hpp"

namespace engine {

void Match::Tick(float delta_time) {
  accumulator_ += delta_time;
  while (accumulator_ >= fixed_dt) {
    // Define how much damping PER SECOND
    const float linear_damping_per_sec = 0.98f;
    const float angular_damping_per_sec = 0.95f;

    if (ball_) {
      ball_->position += fixed_dt * ball_->velocity;
    }
    for (physics::Box& car : boxes_) {
      car.position += fixed_dt * car.velocity;
      glm::quat q = car.rotation;
      glm::vec3 w = car.angular_velocity;
      car.rotation = glm::normalize(q + (0.5f * fixed_dt * q * glm::quat(0, w.x, w.y, w.z)));
    }
    HandleCollisions();

    // Convert to per-frame damping
    float linear_damp = std::pow(linear_damping_per_sec, fixed_dt);
    float angular_damp = std::pow(angular_damping_per_sec, fixed_dt);
    if (ball_) {
      ball_->velocity *= linear_damp;
    }
    for (physics::Box& car : boxes_) {
      car.velocity *= linear_damp;
      car.angular_velocity *= angular_damp;
    }

    accumulator_ -= fixed_dt;
  }
}

void Match::Reset() {
  if (ball_ && initial_ball_) ball_.emplace(*initial_ball_);
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
        LOG_DEBUG("Box/Box contact detected: pentration=%.4f normal=(%.2f, %.2f, %.2f)", contact.penetration,
                  contact.normal.x, contact.normal.y, contact.normal.z);
        for (const auto& p : contact.points) LOG_DEBUG("  point=(%.2f, %.2f, %.2f)", p.x, p.y, p.z);
        LOG_DEBUG("Box before collision: velocity=(%.2f, %.2f, %.2f)", boxes_[i].velocity.x, boxes_[i].velocity.y,
                  boxes_[i].velocity.z);
        LOG_DEBUG("Box before collision: velocity=(%.2f, %.2f, %.2f)", boxes_[j].velocity.x, boxes_[j].velocity.y,
                  boxes_[j].velocity.z);
        physics::Collisions::ResolveElasticCollision(boxes_[i], boxes_[j], contact);
        LOG_DEBUG("Box after collision: velocity=(%.2f, %.2f, %.2f)", boxes_[i].velocity.x, boxes_[i].velocity.y,
                  boxes_[i].velocity.z);
        LOG_DEBUG("Box after collision: velocity=(%.2f, %.2f, %.2f)", boxes_[j].velocity.x, boxes_[j].velocity.y,
                  boxes_[j].velocity.z);
      }
    }
  }
  // Car v Wall collisions
  for (physics::Box& car : boxes_) {
    for (physics::Box& wall : walls_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(car, wall, contact)) {
        LOG_DEBUG("Box/Wall contact detected: pentration=%.4f normal=(%.2f, %.2f, %.2f)", contact.penetration,
                  contact.normal.x, contact.normal.y, contact.normal.z);
        for (const auto& p : contact.points) LOG_DEBUG("  point=(%.2f, %.2f, %.2f)", p.x, p.y, p.z);
        LOG_DEBUG("Box before collision: velocity=(%.2f, %.2f, %.2f)", car.velocity.x, car.velocity.y, car.velocity.z);

        physics::Collisions::ResolveElasticCollision(car, wall, contact);

        LOG_DEBUG("Box after collision: velocity=(%.2f, %.2f, %.2f)", car.velocity.x, car.velocity.y, car.velocity.z);
      }
    }
  }
  // Car v Ground collisions
  for (physics::Box& car : boxes_) {
    physics::Contact contact;
    if (physics::Collisions::ComputeContact(car, *ground_, contact)) {
      LOG_DEBUG("Box/Ground contact detected: pentration=%.4f normal=(%.2f, %.2f, %.2f)", contact.penetration,
                contact.normal.x, contact.normal.y, contact.normal.z);
      for (const auto& p : contact.points) LOG_DEBUG("  point=(%.2f, %.2f, %.2f)", p.x, p.y, p.z);
      LOG_DEBUG("Box before collision: velocity=(%.2f, %.2f, %.2f)", car.velocity.x, car.velocity.y, car.velocity.z);
      physics::Collisions::ResolveElasticCollision(car, *ground_, contact);
      LOG_DEBUG("Box after collision: velocity=(%.2f, %.2f, %.2f)", car.velocity.x, car.velocity.y, car.velocity.z);
    }
  }
}

}  // namespace engine
