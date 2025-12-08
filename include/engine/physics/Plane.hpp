#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

namespace engine::physics
{

class Plane
{
  public:
    glm::vec2 size;
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 normal = glm::vec3(0, 0, 1);
};
} // namespace engine::physics
