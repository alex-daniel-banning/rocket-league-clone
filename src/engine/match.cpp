#include "engine/match.hpp"

#include <glm/gtc/quaternion.hpp>

#include "engine/log.hpp"
#include "engine/physics/box.hpp"
#include "engine/physics/collisions.hpp"
#include "engine/physics/constraint_solver.hpp"
#include "engine/physics/normal_constraint.hpp"

static constexpr glm::vec3 gravity = glm::vec3(0.0f, -9.8f, 0.0f);

namespace engine {

void Match::Tick(float delta_time) {
  accumulator_ += delta_time;
  int substeps = 0;
  while (accumulator_ >= fixed_dt) {
    substeps++;
    ApplyGravity();

    auto constraints = GenerateContactConstraints(fixed_dt);
    constraint_solver_.PreSolve(bodies_, constraints.normal, constraints.friction, fixed_dt);
    constexpr int solver_iterations = 10;
    constraint_solver_.Solve(bodies_, constraints.normal, constraints.friction, solver_iterations);

    if (ball_) {
      ball_->position += fixed_dt * ball_->EffectiveVelocity();
      glm::quat q = ball_->rotation;
      glm::vec3 w = ball_->EffectiveAngularVelocity();
      ball_->rotation = glm::normalize(q + (0.5f * fixed_dt * glm::quat(0, w.x, w.y, w.z) * q));
    }
    for (physics::Box& car : boxes_) {
      car.position += fixed_dt * car.EffectiveVelocity();
      glm::quat q = car.rotation;
      glm::vec3 w = car.EffectiveAngularVelocity();
      car.rotation = glm::normalize(q + (0.5f * fixed_dt * glm::quat(0, w.x, w.y, w.z) * q));
    }

    // --- Damping ---
    const float linear_damping_per_sec = 0.98f;
    const float angular_damping_per_sec = 0.98f;
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
  // LOG_TRACE("tick: dt=%.4f substeps=%d", delta_time, substeps);
}

void Match::Reset() {
  if (ball_ && initial_ball_) ball_.emplace(*initial_ball_);
  boxes_ = {initial_boxes_.begin(), initial_boxes_.end()};
}

void Match::ApplyGravity() {
  for (physics::Box& car : boxes_) {
    car.velocity += fixed_dt * gravity;
  }
  if (ball_) ball_->velocity += fixed_dt * gravity;
}

Match::ContactConstraints Match::GenerateContactConstraints(float dt) {
  ContactConstraints constraints;

  float baumgarte = 0.02f;
  float restitution = 0.3f;
  if (ball_) {
    // Ball v Wall
    for (const physics::Box& wall : walls_) {
      physics::Contact contact;
      if (physics::collisions::ComputeContact(wall, *ball_, contact)) {
        physics::ConstraintSolver::GenerateFromContact(contact, bodies_, dt, constraints.normal, constraints.friction,
                                                       restitution, baumgarte);
      }
    }
    // Ball v Ground
    if (ground_) {
      physics::Contact contact;
      if (physics::collisions::ComputeContact(*ground_, *ball_, contact)) {
        physics::ConstraintSolver::GenerateFromContact(contact, bodies_, dt, constraints.normal, constraints.friction,
                                                       restitution, baumgarte);
      }
    }
    // Ball v Car
    for (const physics::Box& car : boxes_) {
      physics::Contact contact;
      if (physics::collisions::ComputeContact(car, *ball_, contact)) {
        physics::ConstraintSolver::GenerateFromContact(contact, bodies_, dt, constraints.normal, constraints.friction,
                                                       restitution, baumgarte);
      }
    }
  }

  // Car v Car
  if (boxes_.size() > 1) {
    for (unsigned int i = 0; i < boxes_.size() - 1; i++) {
      for (unsigned int j = i + 1; j < boxes_.size(); j++) {
        physics::Contact contact;
        if (physics::collisions::ComputeContact(boxes_[i], boxes_[j], contact)) {
          physics::ConstraintSolver::GenerateFromContact(contact, bodies_, dt, constraints.normal, constraints.friction,
                                                         restitution, baumgarte);
        }
      }
    }
  }
  // Car v Wall
  for (const physics::Box& car : boxes_) {
    for (const physics::Box& wall : walls_) {
      physics::Contact contact;
      if (physics::collisions::ComputeContact(car, wall, contact)) {
        physics::ConstraintSolver::GenerateFromContact(contact, bodies_, dt, constraints.normal, constraints.friction,
                                                       restitution, baumgarte);
      }
    }
  }
  // Car v Ground
  if (ground_) {
    for (const physics::Box& car : boxes_) {
      physics::Contact contact;
      if (physics::collisions::ComputeContact(car, *ground_, contact)) {
        physics::ConstraintSolver::GenerateFromContact(contact, bodies_, dt, constraints.normal, constraints.friction,
                                                       restitution, baumgarte);
      }
    }
  }

  return constraints;
}

}  // namespace engine
