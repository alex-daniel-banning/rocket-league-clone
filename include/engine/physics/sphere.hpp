#pragma once

#include <glm/glm.hpp>

namespace engine::physics {

struct Sphere {
  const float radius = 1.0f;
  const float mass = 1.0f;
  const float mass_inv = 1.0f;
  glm::vec3 position = glm::vec3(0.0f);
  glm::vec3 velocity = glm::vec3(0.0f);
  glm::vec3 angular_velocity = glm::vec3(0.0f);      // unused as of now
  const glm::mat3 inertia_tensor_inv = glm::mat3();  // placeholder for now

  Sphere() = default;
  Sphere(float r, float m, glm::vec3 pos, glm::vec3 vel = glm::vec3(0.0f))
      : radius(r), mass(m), mass_inv(1.0f / m), position(pos), velocity(vel) {}
};

}  // namespace engine::physics
