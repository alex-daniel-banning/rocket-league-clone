#include "engine/physics/contact_constraint_solver.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <tuple>
#include <unordered_map>

#include "engine/physics/contact_constraint.hpp"
#include "glm/ext/vector_float3.hpp"
#include "glm/geometric.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

namespace {

glm::mat3 WorldInverseInertia(const glm::quat& rotation, const glm::mat3& inertia_tensor_inv) {
  glm::mat3 rot = glm::toMat3(rotation);
  return rot * inertia_tensor_inv * glm::transpose(rot);
}

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

void engine::physics::ContactConstraintSolver::GenerateFromContact(const Contact& contact,
                                                                   std::unordered_map<int, Body> bodies, float dt,
                                                                   std::vector<ContactConstraint>& out,
                                                                   float restitution, float baumgarte) {
  glm::vec3 n = contact.normal;
  auto [pos_a, rot_a, inv_i_a, inv_m_a, vel_a, w_a] = std::visit(
      [](auto* body) {
        return std::tuple(body->position, body->rotation, body->inertia_tensor_inv, body->mass_inv, body->velocity,
                          body->angular_velocity);
      },
      bodies[contact.body_a_id]);
  const glm::mat3 i_world_inv_a = WorldInverseInertia(rot_a, inv_i_a);
  auto [pos_b, rot_b, inv_i_b, inv_m_b, vel_b, w_b] = std::visit(
      [](auto* body) {
        return std::tuple(body->position, body->rotation, body->inertia_tensor_inv, body->mass_inv, body->velocity,
                          body->angular_velocity);
      },
      bodies[contact.body_b_id]);
  const glm::mat3 i_world_inv_b = WorldInverseInertia(rot_b, inv_i_b);

  for (const auto& cp : contact.points) {
    glm::vec3 r_a = cp.position - pos_a;
    glm::vec3 r_b = cp.position - pos_b;
    glm::vec3 r_a_cross_n = glm::cross(r_a, n);
    glm::vec3 r_b_cross_n = glm::cross(r_b, n);

    float w = inv_m_a + inv_m_b + glm::dot(glm::cross(i_world_inv_a * r_a_cross_n, r_a), n) +
              glm::dot(glm::cross(i_world_inv_b * r_b_cross_n, r_b), n);

    glm::vec3 v_contact_a = vel_a + glm::cross(w_a, r_a);
    glm::vec3 v_contact_b = vel_b + glm::cross(w_b, r_b);
    float v_n = glm::dot(v_contact_b - v_contact_a, n);

    ContactConstraint cc;
    cc.body_a_id = contact.body_a_id;
    cc.body_b_id = contact.body_b_id;
    cc.jacobian = {-n.x, -n.y, -n.z, -r_a_cross_n.x, -r_a_cross_n.y, -r_a_cross_n.z,
                   n.x,  n.y,  n.z,  r_b_cross_n.x,  r_b_cross_n.y,  r_b_cross_n.z};
    assert(cp.penetration >= 0.0f && "contact point penetration should always be positive");
    float restitution_term = (std::abs(v_n) > 9.8f * dt) ? restitution * v_n : 0.0f;
    float p_slop = 0.002f;
    cc.bias = -(baumgarte / dt) * std::max(0.0f, cp.penetration - p_slop) + restitution_term;
    cc.velocity_bias = restitution_term;  // only if separating?
    cc.position_bias = -(baumgarte / dt) * std::max(cp.penetration - p_slop, 0.0f);
    cc.effective_mass = (w > 0.0f) ? 1.0f / w : 0.0f;
    cc.i_world_inv_a = i_world_inv_a;
    cc.i_world_inv_b = i_world_inv_b;
    cc.accumulated_impulse = 0.0f;
    out.push_back(cc);
  }
}

void engine::physics::ContactConstraintSolver::PreSolve(std::unordered_map<int, Body>& bodies,
                                                        const std::vector<ContactConstraint>& constraints, float dt) {
  // TODO Warm Start
  // apply last frame's impulse
}

void engine::physics::ContactConstraintSolver::Solve(std::unordered_map<int, Body>& bodies,
                                                     std::vector<ContactConstraint>& contact_constraints,
                                                     int iterations) {
  // Reset pseudo velocities
  for (auto& cc : contact_constraints) {
    std::visit(
        [&](auto* body) {
          body->pseudo_velocity = glm::vec3();
          body->pseudo_angular_velocity = glm::vec3();
        },
        bodies[cc.body_a_id]);
    std::visit(
        [&](auto* body) {
          body->pseudo_velocity = glm::vec3();
          body->pseudo_angular_velocity = glm::vec3();
        },
        bodies[cc.body_b_id]);
  }

  for (unsigned int i = 0; i < iterations; i++) {
    for (auto& cc : contact_constraints) {
      float jv = ComputeJV(bodies, cc);
      float lambda = -(jv + cc.velocity_bias) * cc.effective_mass;
      float old_accum = cc.accumulated_impulse;
      cc.accumulated_impulse = std::max(0.0f, old_accum + lambda);
      float delta = cc.accumulated_impulse - old_accum;  // clamped delta
      ApplyImpulse(bodies, cc, delta);

      // Position correction
      float p_jv = ComputePseudoJV(bodies, cc);
      float p_lambda = -(p_jv + cc.position_bias) * cc.effective_mass;
      float old_p_accum = cc.pseudo_accumulated_impulse;
      cc.pseudo_accumulated_impulse = std::max(0.0f, old_p_accum + p_lambda);
      float p_delta = cc.pseudo_accumulated_impulse - old_p_accum;  // clamped delta
      ApplyPseudoImpulse(bodies, cc, p_delta);
    }
  }
}

float engine::physics::ContactConstraintSolver::ComputeJV(std::unordered_map<int, Body>& bodies,
                                                          const ContactConstraint& cc) {
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

float engine::physics::ContactConstraintSolver::ComputePseudoJV(std::unordered_map<int, Body>& bodies,
                                                                const ContactConstraint& cc) {
  // contact normal should point from A to B. Velocity of body A should be negative.
  const int a = cc.body_a_id;
  const int b = cc.body_b_id;
  const glm::vec3 v_a = std::visit([](auto* body) { return body->pseudo_velocity; }, bodies[a]);
  const glm::vec3 w_a = std::visit([](auto* body) { return body->pseudo_angular_velocity; }, bodies[a]);
  const glm::vec3 v_b = std::visit([](auto* body) { return body->pseudo_velocity; }, bodies[b]);
  const glm::vec3 w_b = std::visit([](auto* body) { return body->pseudo_angular_velocity; }, bodies[b]);

  const auto [j_v_a, j_w_a, j_v_b, j_w_b] = UnpackJacobian(cc);

  return glm::dot(j_v_a, v_a) + glm::dot(j_w_a, w_a) + glm::dot(j_v_b, v_b) + glm::dot(j_w_b, w_b);
};

void engine::physics::ContactConstraintSolver::ApplyImpulse(std::unordered_map<int, Body>& bodies,
                                                            const ContactConstraint& cc, float lambda) {
  const int a = cc.body_a_id;
  const int b = cc.body_b_id;

  const auto j = UnpackJacobian(cc);

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

void engine::physics::ContactConstraintSolver::ApplyPseudoImpulse(std::unordered_map<int, Body>& bodies,
                                                                  const ContactConstraint& cc, float lambda) {
  const int a = cc.body_a_id;
  const int b = cc.body_b_id;

  const auto j = UnpackJacobian(cc);

  std::visit(
      [&](auto* body) {
        body->pseudo_velocity += body->mass_inv * j.j_v_a * lambda;
        body->pseudo_angular_velocity += cc.i_world_inv_a * j.j_w_a * lambda;
      },
      bodies[a]);
  std::visit(
      [&](auto* body) {
        body->pseudo_velocity += body->mass_inv * j.j_v_b * lambda;
        body->pseudo_angular_velocity += cc.i_world_inv_b * j.j_w_b * lambda;
      },
      bodies[b]);
}
