#pragma once

#include <engine/physics/Sphere.hpp>
#include <engine/physics/box.hpp>
#include <engine/physics/plane.hpp>
#include <optional>
#include <vector>

namespace engine {
class Match {
 public:
  class Builder {
   public:
    Builder& WithBall(physics::Sphere ball) {
      ball_ = ball;
      return *this;
    }
    Builder& WithGround(physics::Plane ground) {
      ground_ = ground;
      return *this;
    }
    Builder& WithWall(physics::Plane wall) {
      walls_.push_back(wall);
      return *this;
    }
    Builder& WithWalls(std::vector<physics::Plane> walls) {
      walls_ = std::move(walls);
      return *this;
    }
    Builder& WithBox(physics::Box box) {
      boxes_.push_back(box);
      return *this;
    }
    Builder& WithBoxes(std::vector<physics::Box> boxes) {
      boxes_ = std::move(boxes);
      return *this;
    }
    Match Build() { return Match(ball_, ground_, walls_, boxes_); }

   private:
    std::optional<physics::Sphere> ball_;
    std::optional<physics::Plane> ground_;
    std::vector<physics::Plane> walls_;
    std::vector<physics::Box> boxes_;
  };

  const std::optional<physics::Sphere>& GetBall() const { return ball_; }
  const std::vector<physics::Box>& GetBoxes() const { return boxes_; }
  const std::optional<physics::Plane>& GetGround() const { return ground_; }
  const std::vector<physics::Plane>& GetWalls() const { return walls_; }
  void Tick(float delta_time);
  void Reset();

 private:
  Match(std::optional<physics::Sphere> ball, std::optional<physics::Plane> ground, std::vector<physics::Plane> walls,
        std::vector<physics::Box> boxes)
      : ball_(ball), initial_ball_(ball), ground_(ground), boxes_(boxes), initial_boxes_(boxes), walls_(std::move(walls)) {}

  static constexpr float fixed_dt = 1.0f / 120.0f;
  float accumulator_ = 0.0f;
  const std::optional<physics::Sphere> initial_ball_;
  const std::vector<physics::Box> initial_boxes_;
  std::optional<physics::Sphere> ball_;
  std::vector<physics::Box> boxes_;
  std::optional<physics::Plane> ground_;
  std::vector<physics::Plane> walls_;

  void HandleCollisions();
};
}  // namespace engine
