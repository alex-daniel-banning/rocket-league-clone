#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace engine {
class Match;
}

namespace engine::physics {

struct Sphere {
  const float radius = 1.0f;
  const float mass = 1.0f;
  const float mass_inv = 1.0f;
  glm::vec3 position;
  glm::vec3 velocity;
  glm::quat rotation;
  glm::vec3 angular_velocity;
  const glm::mat3 inertia_tensor;
  const glm::mat3 inertia_tensor_inv;

  // Used for split impulse penetration correction
  glm::vec3 pseudo_velocity{0.0f};
  glm::vec3 pseudo_angular_velocity{0.0f};

  glm::vec3 EffectiveVelocity() const { return velocity + pseudo_velocity; }
  glm::vec3 EffectiveAngularVelocity() const { return angular_velocity + pseudo_angular_velocity; }

  Sphere() = delete;
  Sphere(float r, float m, glm::vec3 pos, glm::vec3 vel = glm::vec3(0.0f), glm::quat rot = glm::quat(),
         glm::vec3 ang_vel = glm::vec3())
      : radius(r),
        mass(m),
        mass_inv(1.0f / m),
        position(pos),
        velocity(vel),
        inertia_tensor(GenerateInertiaTensor(m, r)),
        inertia_tensor_inv(GenerateInertiaTensorInverse(m, r)),
        rotation(rot),
        angular_velocity(ang_vel) {}
  int GetId() const { return id_; }

 private:
  int id_ = -1;
  friend class engine::Match;

  static float InertiaScalar(float mass, float radius) { return (2.0f / 5.0f) * mass * radius * radius; }
  static glm::mat3 GenerateInertiaTensor(float mass, float radius) { return glm::mat3(InertiaScalar(mass, radius)); }
  static glm::mat3 GenerateInertiaTensorInverse(float mass, float radius) {
    float i = InertiaScalar(mass, radius);
    float i_inv = (i > 0.0f ? 1.0f / i : 0.0f);
    return glm::mat3(i_inv);
  }
};

}  // namespace engine::physics
