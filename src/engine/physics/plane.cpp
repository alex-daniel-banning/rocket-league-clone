#include <engine/physics/plane.hpp>

namespace engine::physics {
const glm::vec3 Plane::default_normal = glm::vec3(0.0f, 1.0f, 0.0f);

Plane::Plane(float x_length, float z_length, glm::vec3 color,
             glm::vec3 position, glm::quat rotation)
    : x_length_(x_length),
      z_length_(z_length),
      color(color),
      position_(position),
      rotation_(rotation) {
  CalculateCornerPositions();
}

void Plane::CalculateCornerPositions() {
  // clang-format off
  const float hx = x_length_ * 0.5f;
  const float hz = z_length_ * 0.5f;

  corner_positions_[0] = position_ + (rotation_ * glm::vec3(-hx, 0.0f, -hz));
  corner_positions_[1] = position_ + (rotation_ * glm::vec3(hx, 0.0f, -hz));
  corner_positions_[2] = position_ + (rotation_ * glm::vec3(hx, 0.0f, hz));
  corner_positions_[3] = position_ + (rotation_ * glm::vec3(-hx, 0.0f, hz));
  // clang-format on
}

glm::vec3 Plane::GetMin() const {
  glm::vec3 min_pos = corner_positions_[0];
  for (const glm::vec3 p : corner_positions_) {
    min_pos.x = glm::min(min_pos.x, p.x);
    min_pos.y = glm::min(min_pos.y, p.y);
    min_pos.z = glm::min(min_pos.z, p.z);

    // This should behave the same way (component-wise, that is)
    // minPos = glm::min(minPos, p);
  }

  return min_pos;
}

glm::vec3 Plane::GetMax() const {
  glm::vec3 max_pos = corner_positions_[0];
  for (const glm::vec3 p : corner_positions_) {
    max_pos.x = glm::max(max_pos.x, p.x);
    max_pos.y = glm::max(max_pos.y, p.y);
    max_pos.z = glm::max(max_pos.z, p.z);

    // This should behave the same way (component-wise, that is)
    // maxPos = glm::max(maxPos, p);
  }

  return max_pos;
}

void Plane::SetPosition(const glm::vec3 p) {
  position_ = p;
  CalculateCornerPositions();
}

void Plane::SetRotation(const glm::quat r) {
  rotation_ = r;
  CalculateCornerPositions();
}

}  // namespace engine::physics
