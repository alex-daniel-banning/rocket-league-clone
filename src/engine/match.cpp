#include "engine/match.hpp"

#include <cassert>
#include <glm/gtc/quaternion.hpp>
#include <tuple>

#include "engine/log.hpp"
#include "engine/physics/box.hpp"
#include "engine/physics/collisions.hpp"
#include "engine/physics/constraint_solver.hpp"
#include "engine/physics/contact_constraint.hpp"
#include "engine/physics/plane.hpp"

static constexpr glm::vec3 gravity = glm::vec3(0.0f, -9.8f, 0.0f);

namespace {

// TODO, I don't think this belongs here either
glm::mat3 WorldInverseInertia(const glm::quat& rotation, const glm::mat3& inertia_tensor_inv) {
  glm::mat3 rot = glm::toMat3(rotation);
  return rot * inertia_tensor_inv * glm::transpose(rot);
}

// TODO, move into more appropriate class?
void GenerateConstraintsFromContact(const engine::physics::Contact& contact,
                                    std::unordered_map<int, engine::physics::Body> bodies, float dt,
                                    std::vector<ContactConstraint>& out) {
  glm::vec3 n = contact.normal;
  auto [pos_a, rot_a, inv_i_a, inv_m_a] = std::visit(
      [](auto* body) { return std::tuple(body->position, body->rotation, body->inertia_tensor_inv, body->mass_inv); },
      bodies[contact.body_a_id]);
  const glm::mat3 i_world_inv_a = WorldInverseInertia(rot_a, inv_i_a);
  auto [pos_b, rot_b, inv_i_b, inv_m_b] = std::visit(
      [](auto* body) { return std::tuple(body->position, body->rotation, body->inertia_tensor_inv, body->mass_inv); },
      bodies[contact.body_b_id]);
  const glm::mat3 i_world_inv_b = WorldInverseInertia(rot_b, inv_i_b);

  for (const auto& cp : contact.points) {
    glm::vec3 r_a = cp.position - pos_a;
    glm::vec3 r_b = cp.position - pos_b;
    glm::vec3 r_a_cross_n = glm::cross(r_a, n);
    glm::vec3 r_b_cross_n = glm::cross(r_b, n);

    float w = inv_m_a + inv_m_b + glm::dot(glm::cross(i_world_inv_a * r_a_cross_n, r_a), n) +
              glm::dot(glm::cross(i_world_inv_b * r_b_cross_n, r_b), n);

    ContactConstraint cc;
    cc.body_a_id = contact.body_a_id;
    cc.body_b_id = contact.body_b_id;
    cc.jacobian = {-n.x, -n.y, -n.z, -r_a_cross_n.x, -r_a_cross_n.y, -r_a_cross_n.z,
                   n.x,  n.y,  n.z,  r_b_cross_n.x,  r_b_cross_n.y,  r_b_cross_n.z};
    assert(cp.penetration >= 0.0f && "contact point penetration should always be positive");
    float baumgarte = 0.2f;
    cc.bias = -(baumgarte / dt) * cp.penetration;
    cc.effective_mass = (w > 0.0f) ? 1.0f / w : 0.0f;
    cc.i_world_inv_a = i_world_inv_a;
    cc.i_world_inv_b = i_world_inv_b;
    cc.accumulated_impulse = 0.0f;
    out.push_back(cc);
  }
}

}  // namespace

namespace engine {

void Match::Tick(float delta_time) {
  accumulator_ += delta_time;
  int substeps = 0;
  while (accumulator_ >= fixed_dt) {
    substeps++;
    ApplyGravity();

    std::vector<ContactConstraint> contact_constraints = GenerateContactConstraints(fixed_dt);
    // Presolve (warm starting TODO)
    physics::ContactConstraintSolver::PreSolve(bodies_, contact_constraints, fixed_dt);

    // Iteratively solve constraints
    constexpr int solver_iterations = 10;
    physics::ContactConstraintSolver::Solve(bodies_, contact_constraints, solver_iterations);

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

void Match::ApplyGravity() {
  for (physics::Box& car : boxes_) {
    car.velocity += fixed_dt * gravity;
  }
  if (ball_) ball_->velocity += fixed_dt * gravity;
}

std::vector<ContactConstraint> Match::GenerateContactConstraints(float dt) {
  std::vector<ContactConstraint> constraints;

  if (ball_) {
    // Ball v Wall
    for (const physics::Box& wall : walls_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(wall, *ball_, contact)) {
        GenerateConstraintsFromContact(contact, bodies_, dt, constraints);
      }
    }
    // Ball v Ground
    if (ground_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(*ground_, *ball_, contact)) {
        GenerateConstraintsFromContact(contact, bodies_, dt, constraints);
      }
    }
    // Ball v Car
    for (const physics::Box& car : boxes_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(car, *ball_, contact)) {
        GenerateConstraintsFromContact(contact, bodies_, dt, constraints);
      }
    }
  }

  // Car v Car
  if (boxes_.size() > 1) {
    for (unsigned int i = 0; i < boxes_.size() - 1; i++) {
      for (unsigned int j = i + 1; j < boxes_.size(); j++) {
        physics::Contact contact;
        if (physics::Collisions::ComputeContact(boxes_[i], boxes_[j], contact)) {
          GenerateConstraintsFromContact(contact, bodies_, dt, constraints);
        }
      }
    }
  }
  // Car v Wall
  for (const physics::Box& car : boxes_) {
    for (const physics::Box& wall : walls_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(car, wall, contact)) {
        GenerateConstraintsFromContact(contact, bodies_, dt, constraints);
      }
    }
  }
  // Car v Ground
  if (ground_) {
    for (const physics::Box& car : boxes_) {
      physics::Contact contact;
      if (physics::Collisions::ComputeContact(car, *ground_, contact)) {
        GenerateConstraintsFromContact(contact, bodies_, dt, constraints);
      }
    }
  }

  return constraints;
}

}  // namespace engine
