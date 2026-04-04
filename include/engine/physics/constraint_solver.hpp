#pragma once

#include <vector>

#include "engine/physics/contact_constraint.hpp"

namespace engine::physics {

class ConstraintSolver {
 public:
  static void Presolve(std::vector<ContactConstraint> constraints, float dt);
};

}  // namespace engine::physics
