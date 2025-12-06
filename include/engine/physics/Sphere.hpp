#pragma once

#include <glm/glm.hpp>

namespace engine::physics
{

class Sphere
{
  public:
    float radius;
    glm::vec3 position;
    glm::vec3 velocity;
};

} // namespace engine::physics
