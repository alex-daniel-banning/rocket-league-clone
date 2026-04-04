#include "engine/match.hpp"

#include <cassert>
#include <glm/gtc/quaternion.hpp>

#include "engine/log.hpp"
#include "engine/physics/box.hpp"
#include "engine/physics/collisions.hpp"
#include "engine/physics/constraint_solver.hpp"
#include "engine/physics/contact_constraint.hpp"

static constexpr glm::vec3 gravity = glm::vec3(0.0f, -9.8f, 0.0f);

namespace {

// TODO, I don't think this belongs here either
glm::mat3 WorldInverseInertia(const glm::quat& rotation, const glm::mat3& inertia_tensor_inv) {
  glm::mat3 rot = glm::toMat3(rotation);
  return rot * inertia_tensor_inv * glm::transpose(rot);
}

// TODO, move into more appropriate class
void GenerateConstraintsFromContact(const engine::physics::Contact& contact, const glm::vec3& pos_a, float mass_inv_a,
                                    const glm::mat3& i_world_inv_a, const glm::vec3& pos_b, float mass_inv_b,
                                    const glm::mat3& i_world_inv_b, float dt, std::vector<ContactConstraint>& out) {
  glm::vec3 n = contact.normal;
  for (const auto& cp : contact.points) {
    glm::vec3 r_a = cp.position - pos_a;
    glm::vec3 r_b = cp.position - pos_b;
    glm::vec3 r_a_cross_n = glm::cross(r_a, n);
    glm::vec3 r_b_cross_n = glm::cross(r_b, n);

    float w = mass_inv_a + mass_inv_b + glm::dot(glm::cross(i_world_inv_a * r_a_cross_n, r_a), n) +
              glm::dot(glm::cross(i_world_inv_b * r_b_cross_n, r_b), n);

    ContactConstraint cc;
    cc.jacobian = {-n.x, -n.y, -n.z, -r_a_cross_n.x, -r_a_cross_n.y, -r_a_cross_n.z,
                   n.x,  n.y,  n.z,  r_b_cross_n.x,  r_b_cross_n.y,  r_b_cross_n.z};
    assert(cp.penetration >= 0.0f && "contact point penetration should always be positive");
    float baumgarte = 0.2f;
    cc.bias = -(baumgarte / dt) * -cp.penetration;
    cc.effective_mass = (w > 0.0f) ? 1.0f / w : 0.0f;
    cc.accumulated_impulse = 0.0f;
    out.push_back(cc);
  }
}

}  // namespace

namespace engine {

// void Match::Tick(float delta_time) {
//   accumulator_ += delta_time;
//   int substeps = 0;
//   while (accumulator_ >= fixed_dt) {
//     substeps++;
//     ApplyGravity();
//
//     if (ball_) {
//       ball_->position += fixed_dt * ball_->velocity;
//     }
//     for (physics::Box& car : boxes_) {
//       car.position += fixed_dt * car.velocity;
//       glm::quat q = car.rotation;
//       glm::vec3 w = car.angular_velocity;
//       car.rotation = glm::normalize(q + (0.5f * fixed_dt * glm::quat(0, w.x, w.y, w.z) * q));
//     }
//     HandleCollisions();
//
//     // --- Damping ---
//     const float linear_damping_per_sec = 0.98f;
//     const float angular_damping_per_sec = 0.98f;
//     float linear_damp = std::pow(linear_damping_per_sec, fixed_dt);
//     float angular_damp = std::pow(angular_damping_per_sec, fixed_dt);
//     if (ball_) {
//       ball_->velocity *= linear_damp;
//     }
//     for (physics::Box& car : boxes_) {
//       car.velocity *= linear_damp;
//       car.angular_velocity *= angular_damp;
//     }
//
//     accumulator_ -= fixed_dt;
//   }
//   if (substeps > 10) {
//     LOG_WARN("tick death spiral: %d substeps (dt=%.4f)", substeps, delta_time);
//   }
//   LOG_TRACE("tick: dt=%.4f substeps=%d", delta_time, substeps);
// }

void Match::Tick(float delta_time) {
  accumulator_ += delta_time;
  int substeps = 0;
  while (accumulator_ >= fixed_dt) {
    substeps++;
    ApplyGravity();

    std::vector<ContactConstraint> contact_constraints = GenerateContactConstraints(fixed_dt);
    // Presolve (warm starting TODO)
    physics::ConstraintSolver::PreSolve(contact_constraints, fixed_dt);

    // Iteratively solve constraints
    constexpr int solver_iterations = 10;
    physics::ConstraintSolver::Solve(contact_constraints, solver_iterations);

    // Iterate positions
    if (ball_) {
      ball_->position += fixed_dt * ball_->velocity;
    }
    for (physics::Box& car : boxes_) {
      car.position += fixed_dt * car.velocity;
      glm::quat q = car.rotation;
      glm::vec3 w = car.angular_velocity;
      car.rotation = glm::normalize(q + (0.5f * fixed_dt * glm::quat(0, w.x, w.y, w.z) * q));
    }

    /* Leave out damping for now, until I have sequential impulse working.
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
    */

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
      physics::Collisions::HandleCollision(wall, *ball_, 0.5f, 0.3f);
    }
    // Ball v Ground collisions
    if (ground_) {
      physics::Collisions::HandleCollision(*ground_, *ball_, 0.5f, 0.3f);
    }
    // Ball v Car collisions
    for (physics::Box& car : boxes_) {
      physics::Collisions::HandleCollision(car, *ball_, 0.5f, 0.3f);
    }
  }

  // Car v Car collisions
  for (unsigned int i = 0; i < boxes_.size() - 1; i++) {
    for (unsigned int j = i + 1; j < boxes_.size(); j++) {
      physics::Collisions::HandleCollision(boxes_[i], boxes_[j], 0.5f, 0.5f);
    }
  }
  // Car v Wall collisions
  for (physics::Box& car : boxes_) {
    for (physics::Box& wall : walls_) {
      physics::Collisions::HandleCollision(car, wall, 0.5f, 0.5f);
    }
  }
  // Car v Ground collisions
  for (physics::Box& car : boxes_) {
    physics::Collisions::HandleCollision(car, *ground_, 0.5f, 0.7f);
  }
}

void Match::ApplyGravity() {
  for (physics::Box& car : boxes_) {
    car.velocity += fixed_dt * gravity;
  }
  if (ball_) ball_->velocity += fixed_dt * gravity;
}

std::vector<ContactConstraint> Match::GenerateContactConstraints(float dt) {
  std::vector<ContactConstraint> constraints;
  static const glm::mat3 zero_inertia(0.0f);

  if (ball_) {
    // Ball v Wall
    for (const physics::Box& wall : walls_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(wall, *ball_, contact)) {
        glm::mat3 i_wall = WorldInverseInertia(wall.rotation, wall.inertia_tensor_inv);
        GenerateConstraintsFromContact(contact, wall.position, wall.mass_inv, i_wall, ball_->position, ball_->mass_inv,
                                       zero_inertia, dt, constraints);
      }
    }
    // Ball v Ground
    if (ground_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(*ground_, *ball_, contact)) {
        glm::mat3 i_ground = WorldInverseInertia(ground_->rotation, ground_->inertia_tensor_inv);
        GenerateConstraintsFromContact(contact, ground_->position, ground_->mass_inv, i_ground, ball_->position,
                                       ball_->mass_inv, zero_inertia, dt, constraints);
      }
    }
    // Ball v Car
    for (const physics::Box& car : boxes_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(car, *ball_, contact)) {
        glm::mat3 i_car = WorldInverseInertia(car.rotation, car.inertia_tensor_inv);
        GenerateConstraintsFromContact(contact, car.position, car.mass_inv, i_car, ball_->position, ball_->mass_inv,
                                       zero_inertia, dt, constraints);
      }
    }
  }

  // Car v Car
  for (unsigned int i = 0; i < boxes_.size() - 1; i++) {
    for (unsigned int j = i + 1; j < boxes_.size(); j++) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(boxes_[i], boxes_[j], contact)) {
        glm::mat3 i_a = WorldInverseInertia(boxes_[i].rotation, boxes_[i].inertia_tensor_inv);
        glm::mat3 i_b = WorldInverseInertia(boxes_[j].rotation, boxes_[j].inertia_tensor_inv);
        GenerateConstraintsFromContact(contact, boxes_[i].position, boxes_[i].mass_inv, i_a, boxes_[j].position,
                                       boxes_[j].mass_inv, i_b, dt, constraints);
      }
    }
  }
  // Car v Wall
  for (const physics::Box& car : boxes_) {
    for (const physics::Box& wall : walls_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(car, wall, contact)) {
        glm::mat3 i_car = WorldInverseInertia(car.rotation, car.inertia_tensor_inv);
        glm::mat3 i_wall = WorldInverseInertia(wall.rotation, wall.inertia_tensor_inv);
        GenerateConstraintsFromContact(contact, car.position, car.mass_inv, i_car, wall.position, wall.mass_inv, i_wall,
                                       dt, constraints);
      }
    }
  }
  // Car v Ground
  if (ground_) {
    for (const physics::Box& car : boxes_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(car, *ground_, contact)) {
        glm::mat3 i_car = WorldInverseInertia(car.rotation, car.inertia_tensor_inv);
        glm::mat3 i_ground = WorldInverseInertia(ground_->rotation, ground_->inertia_tensor_inv);
        GenerateConstraintsFromContact(contact, car.position, car.mass_inv, i_car, ground_->position, ground_->mass_inv,
                                       i_ground, dt, constraints);
      }
    }
  }

  return constraints;
}

}  // namespace engine
