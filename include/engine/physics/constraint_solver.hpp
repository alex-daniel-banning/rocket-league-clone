#pragma once

#include <unordered_map>
#include <variant>
#include <vector>

#include "engine/physics/box.hpp"
#include "engine/physics/contact_constraint.hpp"
#include "engine/physics/sphere.hpp"

namespace engine::physics {

class ConstraintSolver {
 public:
  void PreSolve(std::vector<ContactConstraint> constraints, float dt);
  void Solve(std::vector<ContactConstraint>& contact_constraints, int iterations);

 private:
  std::unordered_map<int, std::variant<Sphere*, Box*>> bodies_;
  void ApplyImpulse(const ContactConstraint& constraint, float lambda);
};
}  // namespace engine::physics
