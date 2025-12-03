#pragma once
#include <engine/physics/Box.hpp>
#include <engine/physics/Sphere.hpp>
#include <engine/render/Shader.hpp>

namespace engine::render
{

class Renderer
{
  public:
    Renderer();
    void drawBox(const engine::physics::Box &box, engine::render::Shader &shader);
    void drawSphere(const engine::physics::Sphere &sphere, engine::render::Shader &shader);

    // todo, I don't know what these are useful for, but chat gpt suggested
    // void beginFrame();
    // void endFrame();

  private:
    unsigned int cubeVAO, cubeVBO;
};

} // namespace engine::render
