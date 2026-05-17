#pragma once

#include <vector>

#include "engine/physics/body.hpp"
#include "engine/physics/contact.hpp"
#include "engine/physics/contact_constraint.hpp"
#include "engine/physics/friction_constraint.hpp"

namespace engine::physics {

class ContactConstraintSolver {
 public:
  static void GenerateFromContact(const Contact& contact, const std::unordered_map<int, Body>& bodies, float dt,
                                  std::vector<ContactConstraint>& out, std::vector<FrictionConstraint>& friction_out,
                                  float restitution = 1.0f, float baumgarte = 0.2f);
  void PreSolve(std::unordered_map<int, Body>& bodies, std::vector<ContactConstraint>& constraints,
                std::vector<FrictionConstraint>& friction_constraints, float dt);
  void Solve(std::unordered_map<int, Body>& bodies, std::vector<ContactConstraint>& contact_constraints,
             std::vector<FrictionConstraint>& friction_constraints, int iterations);

 private:
  std::vector<ContactConstraint> previous_constraints_;
  std::vector<FrictionConstraint> previous_friction_constraints_;
};
}  // namespace engine::physics
