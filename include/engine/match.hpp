#pragma once

#include <engine/physics/Sphere.hpp>
#include <engine/physics/box.hpp>
#include <engine/physics/plane.hpp>
#include <vector>

namespace engine {
class Match {
 public:
  Match(physics::Sphere s, physics::Plane g, std::vector<physics::Plane> w = {},
        std::vector<physics::Box> b = {})
      : ball_(s),
        initial_ball_(s),
        ground_(g),
        boxes_(b),
        initial_boxes_(b),
        walls_(w) {}

  const physics::Sphere& GetBall() const { return ball_; }
  const std::vector<physics::Box>& GetBoxes() const { return boxes_; }
  const physics::Plane& GetGround() const { return ground_; }
  const std::vector<physics::Plane>& GetWalls() const { return walls_; }
  void Tick(float delta_time);
  void Reset();

 private:
  static constexpr float fixed_dt = 1.0f / 120.0f;
  float accumulator_ = 0.0f;
  const physics::Sphere initial_ball_;
  const std::vector<physics::Box> initial_boxes_;
  physics::Sphere ball_;
  std::vector<physics::Box> boxes_;
  physics::Plane ground_;
  std::vector<physics::Plane> walls_;

  void HandleCollisions();
};
}  // namespace engine
