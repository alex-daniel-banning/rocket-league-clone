#pragma once

#include <variant>

#include "engine/physics/box.hpp"
#include "engine/physics/sphere.hpp"

namespace engine::physics {
using Body = std::variant<Box*, Sphere*>;
}
