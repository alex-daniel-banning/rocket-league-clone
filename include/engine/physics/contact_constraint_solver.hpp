#pragma once

#include <vector>

#include "engine/physics/body.hpp"
#include "engine/physics/contact.hpp"
#include "engine/physics/contact_constraint.hpp"

namespace engine::physics {

class ContactConstraintSolver {
 public:
  static void GenerateFromContact(const Contact& contact, const std::unordered_map<int, Body>& bodies, float dt,
                                  std::vector<ContactConstraint>& out, float restitution = 1.0f,
                                  float baumgarte = 0.2f);
  static void PreSolve(std::unordered_map<int, Body>& bodies, const std::vector<ContactConstraint>& constraints,
                       float dt);
  void Solve(std::unordered_map<int, Body>& bodies, std::vector<ContactConstraint>& contact_constraints,
             int iterations);

 private:
  std::vector<ContactConstraint> previous_constraints_;
  static void ApplyImpulse(std::unordered_map<int, Body>& bodies, const ContactConstraint& constraint, float lambda);
  static void ApplyPseudoImpulse(std::unordered_map<int, Body>& bodies, const ContactConstraint& constraint,
                                 float lambda);
  static float ComputeJV(std::unordered_map<int, Body>& bodies, const ContactConstraint& cc);
  static float ComputePseudoJV(std::unordered_map<int, Body>& bodies, const ContactConstraint& cc);
};
}  // namespace engine::physics
