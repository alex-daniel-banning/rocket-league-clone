#pragma once

#include <unordered_map>
#include <variant>
#include <vector>

#include "engine/physics/box.hpp"
#include "engine/physics/contact_constraint.hpp"
#include "engine/physics/sphere.hpp"

namespace engine::physics {

// Specifically contact constraint solver?
class ConstraintSolver {
 public:
  using Body = std::variant<Box*, Sphere*>;
  explicit ConstraintSolver(std::vector<Body> bodies);
  void PreSolve(std::vector<ContactConstraint> constraints, float dt);
  void Solve(std::vector<ContactConstraint>& contact_constraints, int iterations);

 private:
  int next_body_id_ = 0;
  std::unordered_map<int, Body> bodies_;
  void ApplyImpulse(const ContactConstraint& constraint, float lambda);
  float ComputeJV(ContactConstraint cc);
};
}  // namespace engine::physics
