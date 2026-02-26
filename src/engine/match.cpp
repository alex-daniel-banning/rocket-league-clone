#include "engine/match.hpp"

#include "engine/debug_stepper.hpp"
#include "engine/log.hpp"
#include "engine/physics/box.hpp"
#include "engine/physics/collisions.hpp"

namespace engine {

void Match::Tick(float delta_time) {
  accumulator_ += delta_time;
  int substeps = 0;
  while (accumulator_ >= fixed_dt) {
    if (DebugStepper::pause) return;
    substeps++;
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
      // Before (body-space formula - wrong for world-space w):
      // car.rotation = glm::normalize(q + (0.5f * fixed_dt * q * glm::quat(0, w.x, w.y, w.z)));
      // After (world-space formula):
      car.rotation = glm::normalize(q + (0.5f * fixed_dt * glm::quat(0, w.x, w.y, w.z) * q));
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
  if (substeps > 10) {
    LOG_WARN("tick death spiral: %d substeps (dt=%.4f)", substeps, delta_time);
  }
  LOG_TRACE("tick: dt=%.4f substeps=%d", delta_time, substeps);
}

void Match::Reset() {
  if (ball_ && initial_ball_) ball_.emplace(*initial_ball_);
  boxes_ = {initial_boxes_.begin(), initial_boxes_.end()};
}

void Match::HandleCollisions() {
  if (ball_) {
    // Ball v Wall collisions
    for (physics::Box wall : walls_) {
      physics::Collisions::HandleCollision(wall, *ball_);
    }
    // Ball v Ground collisions
    if (ground_) {
      physics::Collisions::HandleCollision(*ground_, *ball_);
    }
    // Ball v Car collisions
    for (physics::Box& car : boxes_) {
      physics::Collisions::HandleCollision(car, *ball_);
    }
  }

  // Car v Car collisions
  for (unsigned int i = 0; i < boxes_.size() - 1; i++) {
    for (unsigned int j = i + 1; j < boxes_.size(); j++) {
      physics::Collisions::HandleCollision(boxes_[i], boxes_[j]);
    }
  }
  // Car v Wall collisions
  for (physics::Box& car : boxes_) {
    for (physics::Box& wall : walls_) {
      physics::Collisions::HandleCollision(car, wall);
    }
  }
  // Car v Ground collisions
  for (physics::Box& car : boxes_) {
    physics::Collisions::HandleCollision(car, *ground_);
  }
}

}  // namespace engine
