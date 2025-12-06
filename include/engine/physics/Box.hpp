#pragma once
#include <glm/glm.hpp>

namespace engine::physics
{

class Box
{
  public:
    glm::vec3 size;
    glm::vec3 position;
    glm::vec3 velocity;
};
} // namespace engine::physics
