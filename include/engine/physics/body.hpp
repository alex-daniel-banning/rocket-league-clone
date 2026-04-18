#pragma once

#include <ostream>
#include <variant>

#include "engine/physics/box.hpp"
#include "engine/physics/sphere.hpp"

namespace engine::physics {
using Body = std::variant<Box*, Sphere*>;

inline std::ostream& operator<<(std::ostream& os, const Body& body) {
  std::visit(
      [&os](auto* b) {
        os << "Body\n"
           << "  id:       " << b->GetId() << "\n"
           << "  position: (" << b->position.x << ", " << b->position.y << ", " << b->position.z << ")\n"
           << "  velocity: (" << b->velocity.x << ", " << b->velocity.y << ", " << b->velocity.z << ")\n"
           << "  mass:     " << b->mass << "\n"
           << "  mass_inv: " << b->mass_inv << "\n";
      },
      body);
  return os;
}
}  // namespace engine::physics
