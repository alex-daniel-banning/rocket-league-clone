#pragma once

#include <engine/physics/box.hpp>
#include <engine/physics/contact.hpp>
#include <engine/physics/sphere.hpp>

namespace engine::physics::collisions {

bool ComputeContact(const Box& box, const Sphere& sphere, Contact& out);
bool ComputeContact(const Box& box_a, const Box& box_b, Contact& out);

}  // namespace engine::physics::collisions
