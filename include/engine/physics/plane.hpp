#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <engine/render/mesh.hpp>
#include <engine/render/shader.hpp>
#include <glm/gtx/quaternion.hpp>

namespace engine::physics {

class Plane {
 public:
  // temp
  glm::vec3 color;

  Plane() = default;
  explicit Plane(float x_length = 1.0f, float z_length = 1.0f, glm::vec3 color = glm::vec3(0.2f),
                 glm::vec3 position = glm::vec3(0.0f), glm::quat rotation = glm::quat(glm::vec3(0.0f)));
  void CalculateCornerPositions();
  void InitializeRenderData();

  glm::vec3 GetNormal() const { return rotation_ * default_normal; }
  glm::vec3 GetPosition() const { return position_; }
  glm::quat GetRotation() const { return rotation_; }
  float GetXLength() const { return x_length_; }
  float GetZLength() const { return z_length_; }
  std::array<glm::vec3, 4> GetCornerPositions() const { return corner_positions_; }

  void SetPosition(const glm::vec3 p);
  void SetRotation(const glm::quat r);

  glm::vec3 GetMin() const;
  glm::vec3 GetMax() const;

 private:
  // Listed clockwise, starting at the top left position when viewing down the y
  // axis where up is in the -z direction.
  std::array<glm::vec3, 4> corner_positions_;

  float x_length_, z_length_;
  bool initialized_flag_;
  glm::vec3 position_;  // depends on DEFAULT_NORMAL
  glm::quat rotation_;
  static const glm::vec3 default_normal;
};
}  // namespace engine::physics
