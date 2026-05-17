#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include "engine/physics/body.hpp"
#include "engine/physics/box.hpp"
#include "engine/physics/contact_constraint.hpp"
#include "engine/physics/contact_constraint_solver.hpp"
#include "engine/physics/friction_constraint.hpp"
#include "engine/physics/sphere.hpp"

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
  static constexpr float fixed_dt = 1.0f / 120.0f;
  float accumulator_ = 0.0f;
  const std::optional<physics::Sphere> initial_ball_;
  const std::vector<physics::Box> initial_boxes_;
  std::optional<physics::Sphere> ball_;
  std::vector<physics::Box> boxes_;
  std::optional<physics::Box> ground_;
  std::vector<physics::Box> walls_;
  std::unordered_map<int, physics::Body> bodies_;
  physics::ContactConstraintSolver contact_constraint_solver_;

  Match(std::optional<physics::Sphere> ball, std::optional<physics::Box> ground, std::vector<physics::Box> walls,
        std::vector<physics::Box> boxes)
      : ball_(ball),
        initial_ball_(ball),
        ground_(ground),
        boxes_(boxes),
        initial_boxes_(boxes),
        walls_(walls),
        bodies_(BuildBodyMap(ball_, ground_, walls_, boxes_)) {}

  std::unordered_map<int, physics::Body> BuildBodyMap(std::optional<physics::Sphere>& ball,
                                                      std::optional<physics::Box>& ground,
                                                      std::vector<physics::Box>& walls,
                                                      std::vector<physics::Box>& boxes) {
    std::unordered_map<int, physics::Body> bodies;
    int next_id = 0;
    if (ball) {
      ball->id_ = next_id++;
      bodies[ball->GetId()] = &ball.value();
    }
    if (ground) {
      ground->id_ = next_id++;
      bodies[ground->GetId()] = &ground.value();
    }
    for (auto& w : walls) {
      w.id_ = next_id++;
      bodies[w.GetId()] = &w;
    }
    for (auto& b : boxes) {
      b.id_ = next_id++;
      bodies[b.GetId()] = &b;
    }
    return bodies;
  }

  void ApplyGravity();
  struct ContactConstraints {
    std::vector<ContactConstraint> normal;
    std::vector<FrictionConstraint> friction;
  };
  ContactConstraints GenerateContactConstraints(float dt);
};
}  // namespace engine
