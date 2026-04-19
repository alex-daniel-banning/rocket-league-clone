#pragma once
#include <glm/glm.hpp>
#include <ostream>
#include <vector>

namespace engine::physics {

struct ContactPoint {
  glm::vec3 position;
  float penetration;
};

struct Contact {
  glm::vec3 normal;
  std::vector<ContactPoint> points;
  int body_a_id;
  int body_b_id;
};

inline std::ostream& operator<<(std::ostream& os, const Contact& c) {
  os << "Contact\n"
     << "  body_a_id: " << c.body_a_id << "\n"
     << "  body_b_id: " << c.body_b_id << "\n"
     << "  normal:    (" << c.normal.x << ", " << c.normal.y << ", " << c.normal.z << ")\n"
     << "  points (" << c.points.size() << "):\n";
  for (size_t i = 0; i < c.points.size(); i++) {
    const auto& p = c.points[i];
    os << "    [" << i << "] pos=(" << p.position.x << ", " << p.position.y << ", " << p.position.z
       << ") depth=" << p.penetration << "\n";
  }
  return os;
}
}  // namespace engine::physics
