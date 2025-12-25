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
    glm::vec3 position;
    glm::quat rotation;

    Plane(float xsize, float zsize, glm::vec3 color);
    Plane(float xsize, float zsize, glm::vec3 color, glm::vec3 position, glm::quat rotation);

    const render::Mesh &getMesh() const { return mesh; }
    glm::vec3 getNormal() const;

  private:
    // depends on DEFAULT_NORMAL
    engine::render::Mesh mesh;
    static const glm::vec3 DEFAULT_NORMAL;
    static engine::render::Mesh createMesh(float xsize, float zsize, glm::vec3 color);
};
} // namespace engine::physics
