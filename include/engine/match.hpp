#pragma once

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <variant>
#include <vector>

#include "engine/car.hpp"
#include "engine/physics/body.hpp"
#include "engine/physics/box.hpp"
#include "engine/physics/constraint_solver.hpp"
#include "engine/physics/friction_constraint.hpp"
#include "engine/physics/normal_constraint.hpp"
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
    // Adds a chassis box and marks it as driven by a Car controller.
    Builder& WithCar(physics::Box chassis) {
      car_box_indices_.push_back(boxes_.size());
      boxes_.push_back(std::move(chassis));
      return *this;
    }
    Match Build() { return Match(ball_, ground_, walls_, boxes_, car_box_indices_); }

   private:
    std::optional<physics::Sphere> ball_;
    std::optional<physics::Box> ground_;
    std::vector<physics::Box> walls_;
    std::vector<physics::Box> boxes_;
    // Positions within boxes_ that are car chassis (added via WithCar);
    // used after id assignment to build the Car controllers.
    std::vector<std::size_t> car_box_indices_;
  };

  const std::optional<physics::Sphere>& GetBall() const { return ball_; }
  const std::vector<physics::Box>& GetBoxes() const { return boxes_; }
  const std::optional<physics::Box>& GetGround() const { return ground_; }
  const std::vector<physics::Box>& GetWalls() const { return walls_; }
  void Tick(float delta_time);
  void Reset();
  void SetCarInput(CarInput car_input);

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
  std::vector<Car> cars_;  // declared after bodies_: cars reference chassis ids assigned in BuildBodyMap
  physics::ConstraintSolver constraint_solver_;

  Match(std::optional<physics::Sphere> ball, std::optional<physics::Box> ground, std::vector<physics::Box> walls,
        std::vector<physics::Box> boxes, std::vector<std::size_t> car_box_indices)
      : initial_ball_(ball),
        initial_boxes_(boxes),
        ball_(ball),
        boxes_(boxes),
        ground_(ground),
        walls_(walls),
        bodies_(BuildBodyMap(ball_, ground_, walls_, boxes_)),
        cars_(BuildCars(boxes_, car_box_indices)) {}

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

  // Builds the car controllers from the boxes flagged via Builder::WithCar.
  // Must run after BuildBodyMap, which assigns the box ids the cars reference.
  static std::vector<Car> BuildCars(const std::vector<physics::Box>& boxes,
                                    const std::vector<std::size_t>& car_box_indices) {
    std::vector<Car> cars;
    cars.reserve(car_box_indices.size());
    for (std::size_t idx : car_box_indices) cars.emplace_back(boxes[idx].GetId());
    return cars;
  }

  // Resolves a car's chassis id back to the Box owned by boxes_ (via bodies_).
  physics::Box& ResolveChassis(const Car& car) { return *std::get<physics::Box*>(bodies_.at(car.chassis_id())); }

  void ApplyGravity();
  void IntegrateForces();
  struct ContactConstraints {
    std::vector<NormalConstraint> normal;
    std::vector<FrictionConstraint> friction;
  };
  ContactConstraints GenerateContactConstraints(float dt);
};
}  // namespace engine
