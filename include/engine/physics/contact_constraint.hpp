#pragma once

#include <array>
#include <glm/matrix.hpp>
#include <ostream>

struct ContactConstraint {
  long body_a_id;
  long body_b_id;
  std::array<float, 12> jacobian;
  float bias;
  float effective_mass;
  glm::mat3 i_world_inv_a;
  glm::mat3 i_world_inv_b;
  float accumulated_impulse;

  friend std::ostream& operator<<(std::ostream& os, const ContactConstraint& cc) {
    os << "ContactConstraint{" << "bodies=[" << cc.body_a_id << "," << cc.body_b_id << "]" << " bias=" << cc.bias
       << " eff_mass=" << cc.effective_mass << " accum=" << cc.accumulated_impulse << " J=[";
    for (int i = 0; i < 12; i++) {
      if (i > 0) os << ",";
      os << cc.jacobian[i];
    }
    os << "]}";
    return os;
  }
};
