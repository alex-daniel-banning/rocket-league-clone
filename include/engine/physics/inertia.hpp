#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

namespace engine::physics {

// World-space inverse inertia tensor: R · I⁻¹ · Rᵀ, where I⁻¹ is the
// body-space inverse inertia tensor and R is the body's rotation.
inline glm::mat3 WorldInverseInertia(const glm::quat& rotation, const glm::mat3& inertia_tensor_inv) {
  glm::mat3 rot = glm::toMat3(rotation);
  return rot * inertia_tensor_inv * glm::transpose(rot);
}

}  // namespace engine::physics
