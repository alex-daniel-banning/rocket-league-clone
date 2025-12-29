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
    Match(physics::Sphere b, physics::Plane g, std::vector<physics::Plane> w = {})
        : ball(std::move(b)), ground(std::move(g)), walls(std::move(w))
    {
    }

    // Accessors
    const physics::Sphere &getBall() const { return ball; }
    const physics::Plane &getGround() const { return ground; }
    const std::vector<physics::Plane> &getWalls() const { return walls; }
    void tick(float deltaTime);

  private:
    physics::Sphere ball;
    physics::Plane ground;
    std::vector<physics::Plane> walls;

    void handleCollisions();
};
} // namespace engine
