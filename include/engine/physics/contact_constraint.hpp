#pragma once

#include <array>

struct ContactConstraint {
  long body_a_temp_id;
  long body_b_temp_id;
  std::array<float, 12> jacobian;
  float bias;
  float effective_mass;
  float accumulated_impulse;
};
