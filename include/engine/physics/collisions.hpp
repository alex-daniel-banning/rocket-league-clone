#pragma once

#include <engine/physics/box.hpp>
#include <engine/physics/contact.hpp>
#include <engine/physics/sphere.hpp>

namespace engine::physics {

class Collisions {
 public:
  static bool ComputeContact(const Box& box, const Sphere& sphere, Contact& out);
  static void ResolveCollision(Box& box, Sphere& sphere, const Contact& contact, float coefficient_of_restitution);
  static void ResolveElasticCollision(Box& box, Sphere& sphere, const Contact& contact);

  static bool ComputeContact(const Box& box_a, const Box& box_b, Contact& out);
  static void HandleCollision(Box& box_a, Box& box_b, float restitution = 1.0f);
};

}  // namespace engine::physics
