#pragma once
#include <engine/physics/Box.hpp>
#include <engine/physics/Sphere.hpp>
#include <engine/render/Camera.hpp>
#include <engine/render/Shader.hpp>

namespace engine::render
{

class Renderer
{
  public:
    Renderer();
    void drawBox(const engine::physics::Box &box, engine::render::Shader &shader,
                 const Camera &camera);
    void drawBoxWireframe(const engine::physics::Box &box, engine::render::Shader &shader,
                          const Camera &camera);
    void drawSphere(const engine::physics::Sphere &sphere, engine::render::Shader &shader);

    glm::mat4 getProjection(float aspect, float fov = 45.0f);
    glm::mat4 makeModelMatrix(const engine::physics::Box &box);

    // todo, I don't know what these are useful for, but chat gpt suggested
    // void beginFrame();
    // void endFrame();

  private:
    unsigned int cubeVAO, cubeVBO;
    unsigned int cubeWireVAO, cubeWireVBO, cubeWireEBO;
    void initCube();
    void initWireCube();
};

} // namespace engine::render
