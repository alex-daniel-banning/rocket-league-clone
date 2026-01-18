#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace engine::physics
{

struct Box
{
    glm::vec3 size             = glm::vec3(1.0f);
    glm::vec3 position         = glm::vec3(0.0f);
    glm::vec3 velocity         = glm::vec3(0.0f);
    float mass                 = 1.0f;
    glm::quat rotation         = glm::quat(glm::vec3(0.0f));
    glm::vec3 angular_velocity = glm::vec3(0.0f);

    Box() = default;
    Box(glm::vec3 si, glm::vec3 pos = glm::vec3(0.0f), glm::vec3 vel = glm::vec3(0.0f),
        float m = 1.0f, glm::quat r = glm::quat(glm::vec3(0.0f)), glm::vec3 w = glm::vec3())
        : size(si), position(pos), velocity(vel), mass(m), rotation(r), angular_velocity(w)
    {
    }
};
} // namespace engine::physics
