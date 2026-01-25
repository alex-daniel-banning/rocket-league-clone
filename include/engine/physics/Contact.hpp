#pragma once
#include <glm/glm.hpp>

namespace engine::physics {

struct Contact {
  glm::vec3 normal;
  glm::vec3 point;
  float penetration;
};
}  // namespace engine::physics
