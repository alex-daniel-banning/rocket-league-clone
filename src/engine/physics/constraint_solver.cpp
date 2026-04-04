#include "engine/physics/constraint_solver.hpp"

#include <algorithm>

#include "engine/physics/contact_constraint.hpp"
#include "glm/ext/vector_float3.hpp"
#include "glm/geometric.hpp"

namespace {}  // namespace

engine::physics::ConstraintSolver::ConstraintSolver(std::vector<Body> bodies) {
  for (auto& body : bodies) {
    bodies[next_body_id_++] = body;
  }
}

void engine::physics::ConstraintSolver::PreSolve(std::vector<ContactConstraint> constraints, float dt) {
  // TODO Warm Start
  // apply last frame's impulse
}

void engine::physics::ConstraintSolver::Solve(std::vector<ContactConstraint>& contact_constraints, int iterations) {
  for (unsigned int i = 0; i < iterations; i++) {
    for (auto& cc : contact_constraints) {
      float jv = ComputeJV(cc);
      float lambda = -(jv + cc.bias) / cc.effective_mass;
      float old_accum = cc.accumulated_impulse;
      cc.accumulated_impulse = std::max(0.0f, old_accum + lambda);
      float delta = cc.accumulated_impulse - old_accum;  // clamped delta
      ApplyImpulse(cc, delta);
    }
  }
}

float engine::physics::ConstraintSolver::ComputeJV(ContactConstraint cc) {
  //    A = constraint.bodyA
  //    B = constraint.bodyB
  //    J = constraint.jacobian
  //
  //    // JV = J_vA · vA + J_wA · ωA + J_vB · vB + J_wB · ωB
  //    return dot(J.vA, A.velocity) + dot(J.wA, A.angularVelocity) +
  //           dot(J.vB, B.velocity) + dot(J.wB, B.angularVelocity)

  // contact normal should point from A to B. Velocity of body A should be negative.

  const int a = cc.body_a_temp_id;
  const int b = cc.body_b_temp_id;
  const glm::vec3 v_a = std::visit([](auto* body) { return body->velocity; }, bodies_[a]);
  const glm::vec3 w_a = std::visit([](auto* body) { return body->angular_velocity; }, bodies_[a]);
  const glm::vec3 v_b = std::visit([](auto* body) { return body->velocity; }, bodies_[b]);
  const glm::vec3 w_b = std::visit([](auto* body) { return body->angular_velocity; }, bodies_[b]);

  const glm::vec3 j_v_a = glm::vec3(cc.jacobian[0], cc.jacobian[1], cc.jacobian[2]);
  const glm::vec3 j_w_a = glm::vec3(cc.jacobian[3], cc.jacobian[4], cc.jacobian[5]);
  const glm::vec3 j_v_b = glm::vec3(cc.jacobian[6], cc.jacobian[7], cc.jacobian[8]);
  const glm::vec3 j_w_b = glm::vec3(cc.jacobian[9], cc.jacobian[10], cc.jacobian[11]);

  return glm::dot(j_v_a, v_a) + glm::dot(j_w_a, w_a) + glm::dot(j_v_b, v_b) + glm::dot(j_w_b, w_b);
};

void engine::physics::ConstraintSolver::ApplyImpulse(const ContactConstraint& cc, float lambda) {
  const int a = cc.body_a_temp_id;
  const int b = cc.body_b_temp_id;

  const glm::vec3 j_v_a = glm::vec3(cc.jacobian[0], cc.jacobian[1], cc.jacobian[2]);
  const glm::vec3 j_w_a = glm::vec3(cc.jacobian[3], cc.jacobian[4], cc.jacobian[5]);
  const glm::vec3 j_v_b = glm::vec3(cc.jacobian[6], cc.jacobian[7], cc.jacobian[8]);
  const glm::vec3 j_w_b = glm::vec3(cc.jacobian[9], cc.jacobian[10], cc.jacobian[11]);

  std::visit(
      [&](auto* body) {
        body->velocity += body->mass_inv * j_v_a * lambda;
        body->angular_velocity += body->inertia_tensor_inv * j_w_a * lambda;
      },
      bodies_[a]);
  std::visit(
      [&](auto* body) {
        body->velocity += body->mass_inv * j_v_b * lambda;
        body->angular_velocity += body->inertia_tensor_inv * j_w_b * lambda;
      },
      bodies_[b]);
}
