#include "engine/physics/constraint_solver.hpp"

namespace {
std::array<float, 12> ComputeJacobian(const ContactConstraint& constraint) {
  // TODO
  return {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
}

}  // namespace

void engine::physics::ConstraintSolver::Presolve(std::vector<ContactConstraint> constraints, float dt) {
  for (auto& constraint : constraints) {
    constraint.jacobian = ComputeJacobian(constraint);
    positionError = ComputePositionError(constraint);  // Is this just the penetration?
  }
}
