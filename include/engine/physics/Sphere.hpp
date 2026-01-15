#pragma once

#include <glm/glm.hpp>

namespace engine::physics
{

struct Sphere
{
    float radius       = 1.0f;
    float mass         = 1.0f;
    glm::vec3 position = glm::vec3(0.0f);
    glm::vec3 velocity = glm::vec3(0.0f);

    Sphere() = default;
    Sphere(float r, float m, glm::vec3 pos, glm::vec3 vel = glm::vec3(0.0f))
        : radius(r), mass(m), position(pos), velocity(vel)
    {
    }
};

} // namespace engine::physics
