#pragma once
#include <glm/glm.hpp>

namespace engine::physics {

struct ContactPoint {
  glm::vec3 position;
  float penetration;
};

struct Contact {
  glm::vec3 normal;
  std::vector<ContactPoint> points;
  float penetration;  // overall penetration; used by current resolver until PR 2
};
}  // namespace engine::physics
