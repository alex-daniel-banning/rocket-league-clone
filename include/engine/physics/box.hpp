#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace engine::physics {

struct Box {
  glm::vec3 position = glm::vec3(0.0f);
  glm::vec3 velocity = glm::vec3(0.0f);
  const float mass = 1.0f;
  const float mass_inv;
  glm::quat rotation = glm::quat(glm::vec3(0.0f));
  glm::vec3 angular_velocity = glm::vec3(0.0f);
  const glm::mat3 inertia_tensor;
  const glm::mat3 inertia_tensor_inv;

 public:
  const glm::vec3& Size() const { return size_; }
  const glm::vec3& HalfExtents() const { return half_extents_; }

  explicit Box(glm::vec3 si, glm::vec3 pos = glm::vec3(0.0f), glm::vec3 vel = glm::vec3(0.0f), float m = 1.0f,
               glm::quat r = glm::quat(glm::vec3(0.0f)), glm::vec3 w = glm::vec3())
      : size_(si),
        half_extents_(si * 0.5f),
        position(pos),
        velocity(vel),
        mass(m),
        mass_inv(m <= 0.0f ? 0.0f : 1.0f / m),
        inertia_tensor(ComputeInertia(si, m)),
        inertia_tensor_inv(ComputeInertiaInverse(ComputeInertia(si, m))),
        rotation(r),
        angular_velocity(w) {}

 private:
  glm::vec3 size_ = glm::vec3(1.0f);
  glm::vec3 half_extents_ = glm::vec3(0.5f);

  static glm::mat3 ComputeInertia(const glm::vec3& size, float mass) {
    float w = size.x, h = size.y, d = size.z;
    glm::mat3 i(0.0f);
    i[0][0] = (1.0f / 12.0f) * mass * (h * h + d * d);
    i[1][1] = (1.0f / 12.0f) * mass * (w * w + d * d);
    i[2][2] = (1.0f / 12.0f) * mass * (w * w + h * h);
    return i;
  }

  static glm::mat3 ComputeInertiaInverse(const glm::mat3& i) {
    glm::mat3 i_inv(0.0f);
    i_inv[0][0] = i[0][0] == 0.0f ? 0.0f : 1.0f / i[0][0];
    i_inv[1][1] = i[1][1] == 0.0f ? 0.0f : 1.0f / i[1][1];
    i_inv[2][2] = i[2][2] == 0.0f ? 0.0f : 1.0f / i[2][2];
    return i_inv;
  }
};
}  // namespace engine::physics
