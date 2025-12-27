#pragma once

#include <engine/physics/Plane.hpp>
#include <engine/physics/Sphere.hpp>
#include <vector>

namespace engine
{
class Match
{
  public:
    // Constructor
    Match(engine::physics::Sphere b, engine::physics::Plane g,
          std::vector<engine::physics::Plane> w = {})
        : ball(std::move(b)), ground(std::move(g)), walls(std::move(w))
    {
    }

    // Accessors
    const engine::physics::Sphere &getBall() const { return ball; }
    const engine::physics::Plane &getGround() const { return ground; }
    const std::vector<engine::physics::Plane> &getWalls() const { return walls; }
    void tick(float deltaTime);

  private:
    engine::physics::Sphere ball;
    engine::physics::Plane ground;
    std::vector<engine::physics::Plane> walls;
};
} // namespace engine
