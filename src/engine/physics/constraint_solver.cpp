#include "engine/physics/constraint_solver.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <tuple>
#include <unordered_map>

#include "engine/physics/friction_constraint.hpp"
#include "engine/physics/inertia.hpp"
#include "engine/physics/normal_constraint.hpp"

namespace {

struct JacobianComponents {
  glm::vec3 j_v_a;
  glm::vec3 j_w_a;
  glm::vec3 j_v_b;
  glm::vec3 j_w_b;
};

template <typename Constraint>
JacobianComponents UnpackJacobian(const Constraint& c) {
  return {
      glm::vec3(c.jacobian[0], c.jacobian[1], c.jacobian[2]),
      glm::vec3(c.jacobian[3], c.jacobian[4], c.jacobian[5]),
      glm::vec3(c.jacobian[6], c.jacobian[7], c.jacobian[8]),
      glm::vec3(c.jacobian[9], c.jacobian[10], c.jacobian[11]),
  };
}

struct BodyPairKey {
  long body_a_id;
  long body_b_id;
  bool operator==(const BodyPairKey& other) const {
    return body_a_id == other.body_a_id && body_b_id == other.body_b_id;
  }
};

struct BodyPairKeyHash {
  size_t operator()(const BodyPairKey& k) const {
    size_t h = std::hash<long>{}(k.body_a_id);
    h ^= std::hash<long>{}(k.body_b_id) << 1;
    return h;
  }
};

template <typename Constraint>
float ComputeJV(std::unordered_map<int, engine::physics::Body>& bodies, const Constraint& cc) {
  const int a = cc.body_a_id;
  const int b = cc.body_b_id;
  const glm::vec3 v_a = std::visit([](auto* body) { return body->velocity; }, bodies[a]);
  const glm::vec3 w_a = std::visit([](auto* body) { return body->angular_velocity; }, bodies[a]);
  const glm::vec3 v_b = std::visit([](auto* body) { return body->velocity; }, bodies[b]);
  const glm::vec3 w_b = std::visit([](auto* body) { return body->angular_velocity; }, bodies[b]);

  const auto [j_v_a, j_w_a, j_v_b, j_w_b] = UnpackJacobian(cc);

  return glm::dot(j_v_a, v_a) + glm::dot(j_w_a, w_a) + glm::dot(j_v_b, v_b) + glm::dot(j_w_b, w_b);
}

float ComputePseudoJV(std::unordered_map<int, engine::physics::Body>& bodies, const NormalConstraint& cc) {
  const int a = cc.body_a_id;
  const int b = cc.body_b_id;
  const glm::vec3 v_a = std::visit([](auto* body) { return body->pseudo_velocity; }, bodies[a]);
  const glm::vec3 w_a = std::visit([](auto* body) { return body->pseudo_angular_velocity; }, bodies[a]);
  const glm::vec3 v_b = std::visit([](auto* body) { return body->pseudo_velocity; }, bodies[b]);
  const glm::vec3 w_b = std::visit([](auto* body) { return body->pseudo_angular_velocity; }, bodies[b]);

  const auto [j_v_a, j_w_a, j_v_b, j_w_b] = UnpackJacobian(cc);

  return glm::dot(j_v_a, v_a) + glm::dot(j_w_a, w_a) + glm::dot(j_v_b, v_b) + glm::dot(j_w_b, w_b);
}

template <typename Constraint>
void ApplyImpulse(std::unordered_map<int, engine::physics::Body>& bodies, const Constraint& c, float lambda) {
  const int a = c.body_a_id;
  const int b = c.body_b_id;

  const auto j = UnpackJacobian(c);

  std::visit(
      [&](auto* body) {
        body->velocity += body->mass_inv * j.j_v_a * lambda;
        body->angular_velocity += c.i_world_inv_a * j.j_w_a * lambda;
      },
      bodies[a]);
  std::visit(
      [&](auto* body) {
        body->velocity += body->mass_inv * j.j_v_b * lambda;
        body->angular_velocity += c.i_world_inv_b * j.j_w_b * lambda;
      },
      bodies[b]);
}

void ApplyPseudoImpulse(std::unordered_map<int, engine::physics::Body>& bodies, const NormalConstraint& cc,
                        float lambda) {
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

glm::vec3 ArbitraryPerpendicular(const glm::vec3& n) {
  if (std::abs(n.x) < 0.9f) {
    return glm::normalize(glm::cross(n, glm::vec3(1, 0, 0)));
  }
  return glm::normalize(glm::cross(n, glm::vec3(0, 1, 0)));
}

// Carries each constraint's accumulated impulse over from the previous frame by matching it to the
// closest previous constraint on the same body pair, then seeds the solver by applying that impulse.
// `matches` is an extra predicate for constraint-type-specific matching (e.g. friction tangent index).
template <typename Constraint, typename MatchPred>
void WarmStart(std::unordered_map<int, engine::physics::Body>& bodies, std::vector<Constraint>& constraints,
               const std::vector<Constraint>& previous, MatchPred matches) {
  std::unordered_map<BodyPairKey, std::vector<int>, BodyPairKeyHash> lookup;
  for (int idx = 0; idx < static_cast<int>(previous.size()); idx++) {
    const Constraint& c = previous[idx];
    lookup[{std::min(c.body_a_id, c.body_b_id), std::max(c.body_a_id, c.body_b_id)}].push_back(idx);
  }

  const float epsilon = 0.01f;
  for (auto& c : constraints) {
    c.accumulated_impulse = 0.0f;
    auto it = lookup.find({std::min(c.body_a_id, c.body_b_id), std::max(c.body_a_id, c.body_b_id)});
    if (it == lookup.end()) continue;

    int best = -1;
    float best_dist = epsilon * epsilon;
    for (int idx : it->second) {
      if (!matches(c, previous[idx])) continue;
      float dist = glm::distance2(c.position, previous[idx].position);
      if (dist < best_dist) {
        best = idx;
        best_dist = dist;
      }
    }
    if (best >= 0) {
      c.accumulated_impulse = previous[best].accumulated_impulse;
      ApplyImpulse(bodies, c, c.accumulated_impulse);
    }
  }
}

}  // namespace

void engine::physics::ConstraintSolver::GenerateFromContact(const Contact& contact,
                                                            const std::unordered_map<int, Body>& bodies, float dt,
                                                            std::vector<NormalConstraint>& normal_out,
                                                            std::vector<FrictionConstraint>& friction_out,
                                                            float restitution, float baumgarte) {
  glm::vec3 n = contact.normal;
  auto [pos_a, rot_a, inv_i_a, inv_m_a, vel_a, w_a] = std::visit(
      [](auto* body) {
        return std::tuple(body->position, body->rotation, body->inertia_tensor_inv, body->mass_inv, body->velocity,
                          body->angular_velocity);
      },
      bodies.at(contact.body_a_id));
  const glm::mat3 i_world_inv_a = WorldInverseInertia(rot_a, inv_i_a);
  auto [pos_b, rot_b, inv_i_b, inv_m_b, vel_b, w_b] = std::visit(
      [](auto* body) {
        return std::tuple(body->position, body->rotation, body->inertia_tensor_inv, body->mass_inv, body->velocity,
                          body->angular_velocity);
      },
      bodies.at(contact.body_b_id));
  const glm::mat3 i_world_inv_b = WorldInverseInertia(rot_b, inv_i_b);

  float mu_a = std::visit([](auto* body) { return body->friction; }, bodies.at(contact.body_a_id));
  float mu_b = std::visit([](auto* body) { return body->friction; }, bodies.at(contact.body_b_id));
  float mu = std::sqrt(mu_a * mu_b);

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

    NormalConstraint cc;
    cc.body_a_id = contact.body_a_id;
    cc.body_b_id = contact.body_b_id;
    cc.jacobian = {-n.x, -n.y, -n.z, -r_a_cross_n.x, -r_a_cross_n.y, -r_a_cross_n.z,
                   n.x,  n.y,  n.z,  r_b_cross_n.x,  r_b_cross_n.y,  r_b_cross_n.z};
    assert(cp.penetration >= 0.0f && "contact point penetration should always be positive");
    float restitution_term = (std::abs(v_n) > 9.8f * dt) ? restitution * v_n : 0.0f;
    float p_slop = 0.002f;
    cc.velocity_bias = restitution_term;
    cc.position_bias = -(baumgarte / dt) * std::max(cp.penetration - p_slop, 0.0f);
    cc.effective_mass = (w > 0.0f) ? 1.0f / w : 0.0f;
    cc.i_world_inv_a = i_world_inv_a;
    cc.i_world_inv_b = i_world_inv_b;
    cc.accumulated_impulse = 0.0f;
    cc.position = cp.position;
    int normal_index = static_cast<int>(normal_out.size());
    normal_out.push_back(cc);

    if (mu <= 0.0f) continue;

    glm::vec3 v_rel = v_contact_b - v_contact_a;
    glm::vec3 v_tangent = v_rel - glm::dot(v_rel, n) * n;
    glm::vec3 t1;
    if (glm::length2(v_tangent) > 1e-8f) {
      t1 = glm::normalize(v_tangent);
    } else {
      t1 = ArbitraryPerpendicular(n);
    }
    glm::vec3 t2 = glm::cross(n, t1);

    glm::vec3 tangents[2] = {t1, t2};
    for (int ti = 0; ti < 2; ti++) {
      const glm::vec3& t = tangents[ti];
      glm::vec3 r_a_cross_t = glm::cross(r_a, t);
      glm::vec3 r_b_cross_t = glm::cross(r_b, t);

      float w_t = inv_m_a + inv_m_b + glm::dot(glm::cross(i_world_inv_a * r_a_cross_t, r_a), t) +
                  glm::dot(glm::cross(i_world_inv_b * r_b_cross_t, r_b), t);

      FrictionConstraint fc;
      fc.body_a_id = contact.body_a_id;
      fc.body_b_id = contact.body_b_id;
      fc.jacobian = {-t.x, -t.y, -t.z, -r_a_cross_t.x, -r_a_cross_t.y, -r_a_cross_t.z,
                     t.x,  t.y,  t.z,  r_b_cross_t.x,  r_b_cross_t.y,  r_b_cross_t.z};
      fc.effective_mass = (w_t > 0.0f) ? 1.0f / w_t : 0.0f;
      fc.i_world_inv_a = i_world_inv_a;
      fc.i_world_inv_b = i_world_inv_b;
      fc.accumulated_impulse = 0.0f;
      fc.mu = mu;
      fc.position = cp.position;
      fc.normal_constraint_index = normal_index;
      fc.tangent_index = ti;
      friction_out.push_back(fc);
    }
  }
}

void engine::physics::ConstraintSolver::PreSolve(std::unordered_map<int, Body>& bodies,
                                                 std::vector<NormalConstraint>& normal_constraints,
                                                 std::vector<FrictionConstraint>& friction_constraints, float dt) {
  // Normal constraints: any constraint on the same body pair is a candidate match.
  WarmStart(bodies, normal_constraints, previous_normal_constraints_,
            [](const NormalConstraint&, const NormalConstraint&) { return true; });

  // Friction constraints: additionally require the same tangent direction.
  WarmStart(bodies, friction_constraints, previous_friction_constraints_,
            [](const FrictionConstraint& a, const FrictionConstraint& b) {
              return a.tangent_index == b.tangent_index;
            });
}

void engine::physics::ConstraintSolver::Solve(std::unordered_map<int, Body>& bodies,
                                              std::vector<NormalConstraint>& normal_constraints,
                                              std::vector<FrictionConstraint>& friction_constraints, int iterations) {
  // Reset pseudo velocities
  for (auto& cc : normal_constraints) {
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
    // Solve normal constraints
    for (auto& cc : normal_constraints) {
      float jv = ComputeJV(bodies, cc);
      float lambda = -(jv + cc.velocity_bias) * cc.effective_mass;
      float old_accum = cc.accumulated_impulse;
      cc.accumulated_impulse = std::max(0.0f, old_accum + lambda);
      float delta = cc.accumulated_impulse - old_accum;
      ApplyImpulse(bodies, cc, delta);

      // Position correction
      float p_jv = ComputePseudoJV(bodies, cc);
      float p_lambda = -(p_jv + cc.position_bias) * cc.effective_mass;
      float old_p_accum = cc.pseudo_accumulated_impulse;
      cc.pseudo_accumulated_impulse = std::max(0.0f, old_p_accum + p_lambda);
      float p_delta = cc.pseudo_accumulated_impulse - old_p_accum;
      ApplyPseudoImpulse(bodies, cc, p_delta);
    }

    // Solve friction constraints
    for (auto& fc : friction_constraints) {
      float jv = ComputeJV(bodies, fc);
      float lambda = -jv * fc.effective_mass;
      const auto& nc = normal_constraints[fc.normal_constraint_index];
      float max_friction = fc.mu * (nc.accumulated_impulse + nc.pseudo_accumulated_impulse);
      float old_accum = fc.accumulated_impulse;
      fc.accumulated_impulse = std::clamp(old_accum + lambda, -max_friction, max_friction);
      float delta = fc.accumulated_impulse - old_accum;
      ApplyImpulse(bodies, fc, delta);
    }
  }

  previous_normal_constraints_ = normal_constraints;
  previous_friction_constraints_ = friction_constraints;
}
