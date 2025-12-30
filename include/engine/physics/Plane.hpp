#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include <engine/render/Mesh.hpp>
#include <engine/render/Shader.hpp>

namespace engine::physics
{

class Plane
{
  public:
    // temp
    glm::vec3 color;

    Plane() = default;
    Plane(float xLength = 1.0f, float zLength = 1.0f, glm::vec3 color = glm::vec3(0.2f),
          glm::vec3 position = glm::vec3(0.0f), glm::quat rotation = glm::quat(glm::vec3(0.0f)));
    void calculateCornerPositions();
    void initializeRenderData();

    // const render::Mesh *getMesh() const;
    glm::vec3 getNormal() const;
    glm::vec3 getMin() const;
    glm::vec3 getMax() const;
    glm::vec3 getPosition() const;
    void setPosition(const glm::vec3 p);
    glm::quat getRotation() const;
    void setRotation(const glm::quat r);
    float getXLength() const;
    float getZLength() const;
    std::array<glm::vec3, 4> getCornerPositions() const;

  private:
    // Listed clockwise, starting at the top left position when viewing down the y axis where up is
    // in the -z direction.
    std::array<glm::vec3, 4> cornerPositions;

    float xLength, zLength;
    bool initializedFlag;
    glm::vec3 position; // depends on DEFAULT_NORMAL
    glm::quat rotation;
    static const glm::vec3 DEFAULT_NORMAL;
};
} // namespace engine::physics
