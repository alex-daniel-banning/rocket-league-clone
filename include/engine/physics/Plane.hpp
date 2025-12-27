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
    glm::vec2 size;

    // Listed clockwise, starting at the top left position when viewing down the y axis where up is
    // in the -z direction.
    std::array<glm::vec3, 4> cornerPositions;

    // temp
    glm::vec3 color;

    Plane(float xsize, float zsize, glm::vec3 color);
    Plane(float xsize, float zsize, glm::vec3 color, glm::vec3 position, glm::quat rotation);
    void calculateCornerPositions();
    void initializeRenderData();

    const render::Mesh *getMesh() const;
    glm::vec3 getNormal() const;
    glm::vec3 getMin() const;
    glm::vec3 getMax() const;
    glm::vec3 getPosition() const;
    void setPosition(const glm::vec3 p);
    glm::quat getRotation() const;
    void setRotation(const glm::quat r);

  private:
    // depends on DEFAULT_NORMAL
    bool initializedFlag;
    glm::vec3 position;
    glm::quat rotation;
    engine::render::Mesh *mesh;
    static const glm::vec3 DEFAULT_NORMAL;
    static engine::render::Mesh createMesh(float xsize, float zsize, glm::vec3 color);
};
} // namespace engine::physics
