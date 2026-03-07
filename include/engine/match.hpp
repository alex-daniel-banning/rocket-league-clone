#pragma once

#include <engine/physics/box.hpp>
#include <engine/physics/plane.hpp>
#include <engine/physics/sphere.hpp>
#include <optional>
#include <vector>

namespace engine {
class Match {
 public:
  class Builder {
   public:
    Builder& WithBall(physics::Sphere ball) {
      ball_.emplace(std::move(ball));
      return *this;
    }
    Builder& WithGround(physics::Box ground) {
      ground_.emplace(std::move(ground));
      return *this;
    }
    Builder& WithWall(physics::Box wall) {
      walls_.push_back(std::move(wall));
      return *this;
    }
    Builder& WithWalls(std::vector<physics::Box> walls) {
      walls_ = std::move(walls);
      return *this;
    }
    Builder& WithBox(physics::Box box) {
      boxes_.push_back(std::move(box));
      return *this;
    }
    Builder& WithBoxes(std::vector<physics::Box> boxes) {
      boxes_ = std::move(boxes);
      return *this;
    }
    Match Build() { return Match(ball_, ground_, walls_, boxes_); }

   private:
    std::optional<physics::Sphere> ball_;
    std::optional<physics::Box> ground_;
    std::vector<physics::Box> walls_;
    std::vector<physics::Box> boxes_;
  };

  const std::optional<physics::Sphere>& GetBall() const { return ball_; }
  const std::vector<physics::Box>& GetBoxes() const { return boxes_; }
  const std::optional<physics::Box>& GetGround() const { return ground_; }
  const std::vector<physics::Box>& GetWalls() const { return walls_; }
  void Tick(float delta_time);
  void Reset();

 private:
  Match(std::optional<physics::Sphere> ball, std::optional<physics::Box> ground, std::vector<physics::Box> walls,
        std::vector<physics::Box> boxes)
      : ball_(ball), initial_ball_(ball), ground_(ground), boxes_(boxes), initial_boxes_(boxes), walls_(walls) {}

  static constexpr float fixed_dt = 1.0f / 120.0f;
  float accumulator_ = 0.0f;
  const std::optional<physics::Sphere> initial_ball_;
  const std::vector<physics::Box> initial_boxes_;
  std::optional<physics::Sphere> ball_;
  std::vector<physics::Box> boxes_;
  std::optional<physics::Box> ground_;
  std::vector<physics::Box> walls_;

  void HandleCollisions();
  void ApplyGravity();
};
}  // namespace engine
