#pragma once

#include <engine/physics/Box.hpp>
#include <engine/physics/Plane.hpp>
#include <engine/physics/Sphere.hpp>
#include <vector>

namespace engine {
class Match {
 public:
  Match(physics::Sphere s, physics::Plane g, std::vector<physics::Plane> w = {},
        std::vector<physics::Box> b = {})
      : ball(s),
        initialBall(s),
        ground(g),
        boxes(b),
        initialBoxes(b),
        walls(w) {}

  const physics::Sphere &getBall() const { return ball; }
  const std::vector<physics::Box> &getBoxes() const { return boxes; }
  const physics::Plane &getGround() const { return ground; }
  const std::vector<physics::Plane> &getWalls() const { return walls; }
  void tick(float deltaTime);
  void reset();

 private:
  static constexpr float FIXED_DT = 1.0f / 120.0f;
  float accumulator = 0.0f;
  const physics::Sphere initialBall;
  const std::vector<physics::Box> initialBoxes;
  physics::Sphere ball;
  std::vector<physics::Box> boxes;
  physics::Plane ground;
  std::vector<physics::Plane> walls;

  void handleCollisions();
};
}  // namespace engine
