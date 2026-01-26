#pragma once

#include <engine/physics/Contact.hpp>
#include <engine/physics/Sphere.hpp>
#include <engine/physics/box.hpp>
#include <engine/physics/plane.hpp>

namespace engine::physics {

class Collisions {
 public:
  static bool Collides(const Plane& plane, Sphere sphere);
  static void HandleElasticCollision(const Plane& plane, Sphere& sphere);
  static float DistanceSquared(const Plane& plane, Sphere sphere);

  static bool ComputeContact(const Box& box, const Sphere& sphere,
                             Contact& out);
  static void ResolveCollision(Box& box, Sphere& sphere, Contact contact,
                               float coefficient_of_restitution);
  static void ResolveElasticCollision(Box& box, Sphere& sphere,
                                      Contact contact);

  static bool ComputeContact(const Box& box_a, const Box& box_b, Contact& out);
  static void ResolveCollision(Box& box_a, Box& box_b, Contact contact,
                               float coefficient_of_restitution);

 private:
  static std::array<glm::vec3, 3> GetAxesFromQuaternion(glm::quat q);
  static float CalculateOverlap(glm::vec3 axis, const Box& box_a,
                                const Box& box_b,
                                const std::array<glm::vec3, 3> axes_a,
                                const std::array<glm::vec3, 3> axes_b);
};

}  // namespace engine::physics
