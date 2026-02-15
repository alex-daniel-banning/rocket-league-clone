#pragma once
#include <glm/glm.hpp>

namespace engine::physics {

struct Contact {
  glm::vec3 normal;
  std::vector<glm::vec3> points;
  float penetration;
};
}  // namespace engine::physics
