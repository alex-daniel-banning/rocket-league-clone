#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <engine/render/Mesh.hpp>
#include <engine/render/Shader.hpp>
#include <glm/gtx/quaternion.hpp>

namespace engine::physics {

class Plane {
 public:
  // temp
  glm::vec3 color;

  Plane() = default;
  Plane(float xLength = 1.0f, float zLength = 1.0f,
        glm::vec3 color = glm::vec3(0.2f), glm::vec3 position = glm::vec3(0.0f),
        glm::quat rotation = glm::quat(glm::vec3(0.0f)));
  void calculateCornerPositions();
  void initializeRenderData();

  glm::vec3 getNormal() const { return rotation * DEFAULT_NORMAL; }
  glm::vec3 getPosition() const { return position; }
  glm::quat getRotation() const { return rotation; }
  float getXLength() const { return xLength; }
  float getZLength() const { return zLength; }
  std::array<glm::vec3, 4> getCornerPositions() const {
    return cornerPositions;
  }

  void setPosition(const glm::vec3 p);
  void setRotation(const glm::quat r);

  glm::vec3 getMin() const;
  glm::vec3 getMax() const;

 private:
  // Listed clockwise, starting at the top left position when viewing down the y
  // axis where up is in the -z direction.
  std::array<glm::vec3, 4> cornerPositions;

  float xLength, zLength;
  bool initializedFlag;
  glm::vec3 position;  // depends on DEFAULT_NORMAL
  glm::quat rotation;
  static const glm::vec3 DEFAULT_NORMAL;
};
}  // namespace engine::physics
