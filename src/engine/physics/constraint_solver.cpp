#include <algorithm>
#include <unordered_map>

#include "engine/physics/contact_constraint.hpp"
#include "engine/physics/contact_constraint_solver.hpp"
#include "glm/ext/vector_float3.hpp"
#include "glm/geometric.hpp"

namespace {

struct JacobianComponents {
  glm::vec3 j_v_a;
  glm::vec3 j_w_a;
  glm::vec3 j_v_b;
  glm::vec3 j_w_b;
};

JacobianComponents UnpackJacobian(const ContactConstraint& cc) {
  return {
      glm::vec3(cc.jacobian[0], cc.jacobian[1], cc.jacobian[2]),
      glm::vec3(cc.jacobian[3], cc.jacobian[4], cc.jacobian[5]),
      glm::vec3(cc.jacobian[6], cc.jacobian[7], cc.jacobian[8]),
      glm::vec3(cc.jacobian[9], cc.jacobian[10], cc.jacobian[11]),
  };
}

}  // namespace

void engine::physics::ContactConstraintSolver::PreSolve(std::unordered_map<int, Body>& bodies,
                                                        const std::vector<ContactConstraint>& constraints, float dt) {
  // TODO Warm Start
  // apply last frame's impulse
}

void engine::physics::ContactConstraintSolver::Solve(std::unordered_map<int, Body>& bodies,
                                                     std::vector<ContactConstraint>& contact_constraints,
                                                     int iterations) {
  for (unsigned int i = 0; i < iterations; i++) {
    for (auto& cc : contact_constraints) {
      float jv = ComputeJV(bodies, cc);
      float lambda = -(jv + cc.bias) / cc.effective_mass;
      float old_accum = cc.accumulated_impulse;
      cc.accumulated_impulse = std::max(0.0f, old_accum + lambda);
      float delta = cc.accumulated_impulse - old_accum;  // clamped delta
      ApplyImpulse(bodies, cc, delta);
    }
  }
}

float engine::physics::ContactConstraintSolver::ComputeJV(std::unordered_map<int, Body>& bodies,
                                                          const ContactConstraint& cc) {
  //    A = constraint.bodyA
  //    B = constraint.bodyB
  //    J = constraint.jacobian
  //
  //    // JV = J_vA · vA + J_wA · ωA + J_vB · vB + J_wB · ωB
  //    return dot(J.vA, A.velocity) + dot(J.wA, A.angularVelocity) +
  //           dot(J.vB, B.velocity) + dot(J.wB, B.angularVelocity)

  // contact normal should point from A to B. Velocity of body A should be negative.

  const int a = cc.body_a_id;
  const int b = cc.body_b_id;
  const glm::vec3 v_a = std::visit([](auto* body) { return body->velocity; }, bodies[a]);
  const glm::vec3 w_a = std::visit([](auto* body) { return body->angular_velocity; }, bodies[a]);
  const glm::vec3 v_b = std::visit([](auto* body) { return body->velocity; }, bodies[b]);
  const glm::vec3 w_b = std::visit([](auto* body) { return body->angular_velocity; }, bodies[b]);

  const auto [j_v_a, j_w_a, j_v_b, j_w_b] = UnpackJacobian(cc);

  return glm::dot(j_v_a, v_a) + glm::dot(j_w_a, w_a) + glm::dot(j_v_b, v_b) + glm::dot(j_w_b, w_b);
};

void engine::physics::ContactConstraintSolver::ApplyImpulse(std::unordered_map<int, Body>& bodies,
                                                            const ContactConstraint& cc, float lambda) {
  const int a = cc.body_a_id;
  const int b = cc.body_b_id;

  const JacobianComponents j = UnpackJacobian(cc);

  std::visit(
      [&](auto* body) {
        body->velocity += body->mass_inv * j.j_v_a * lambda;
        body->angular_velocity += cc.i_world_inv_a * j.j_w_a * lambda;
      },
      bodies[a]);
  std::visit(
      [&](auto* body) {
        body->velocity += body->mass_inv * j.j_v_b * lambda;
        body->angular_velocity += cc.i_world_inv_b * j.j_w_b * lambda;
      },
      bodies[b]);
}
