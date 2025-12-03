#pragma once
#include <engine/physics/Box.hpp>
#include <engine/physics/Sphere.hpp>

namespace engine::render
{

class Renderer
{
  public:
    void drawBox(const engine::physics::Box &box);
    void drawSphere(const engine::physics::Sphere &sphere);

    // todo, I don't know what these are useful for, but chat gpt suggested
    // void beginFrame();
    // void endFrame();
};

} // namespace engine::render
