#pragma once

#include <vector>

#include "engine/physics/body.hpp"
#include "engine/physics/contact_constraint.hpp"

namespace engine::physics {

// Specifically contact constraint solver?
class ContactConstraintSolver {
 public:
  static void PreSolve(std::unordered_map<int, Body>& bodies, const std::vector<ContactConstraint>& constraints,
                       float dt);
  static void Solve(std::unordered_map<int, Body>& bodies, std::vector<ContactConstraint>& contact_constraints,
                    int iterations);

 private:
  static void ApplyImpulse(std::unordered_map<int, Body>& bodies, const ContactConstraint& constraint, float lambda);
  static float ComputeJV(std::unordered_map<int, Body>& bodies, const ContactConstraint& cc);
};
}  // namespace engine::physics
