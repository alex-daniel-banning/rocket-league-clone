#pragma once

#include <array>
#include <glm/matrix.hpp>

struct ContactConstraint {
  long body_a_id;
  long body_b_id;
  std::array<float, 12> jacobian;
  float bias;
  float effective_mass;
  glm::mat3 i_world_inv_a;
  glm::mat3 i_world_inv_b;
  float accumulated_impulse;
};
