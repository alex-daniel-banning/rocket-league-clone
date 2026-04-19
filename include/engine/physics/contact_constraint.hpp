#pragma once

#include <array>
#include <glm/matrix.hpp>
#include <ostream>

struct ContactConstraint {
  long body_a_id;
  long body_b_id;
  std::array<float, 12> jacobian;
  float effective_mass;
  glm::mat3 i_world_inv_a;
  glm::mat3 i_world_inv_b;
  float accumulated_impulse = 0.0f;
  float pseudo_accumulated_impulse = 0.0f;
  float velocity_bias;  // restitution bounce only
  float position_bias;  // Baumgarte penetration correction only

  friend std::ostream& operator<<(std::ostream& os, const ContactConstraint& cc) {
    os << "ContactConstraint{\n"
       << "  bodies=[" << cc.body_a_id << "," << cc.body_b_id << "]\n"
       << "  effective_mass=" << cc.effective_mass << "\n"
       << "  accumulated_impulse=" << cc.accumulated_impulse << "\n"
       << "  pseudo_accumulated_impulse=" << cc.pseudo_accumulated_impulse << "\n"
       << "  velocity_bias=" << cc.velocity_bias << "\n"
       << "  position_bias=" << cc.position_bias << "\n"
       << "  J=[";
    for (int i = 0; i < 12; i++) {
      if (i > 0) {
        os << ",";
      }
      os << cc.jacobian[i];
    }
    os << "]\n}";
    return os;
  }
};
