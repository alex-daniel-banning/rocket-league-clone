#pragma once

#include <array>
#include <glm/matrix.hpp>

struct FrictionConstraint {
  long body_a_id;
  long body_b_id;
  std::array<float, 12> jacobian;
  float effective_mass;
  glm::mat3 i_world_inv_a;
  glm::mat3 i_world_inv_b;
  float accumulated_impulse = 0.0f;
  float mu;
  glm::vec3 position;
  int normal_constraint_index;
};
