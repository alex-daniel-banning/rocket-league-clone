#pragma once

#include <engine/physics/Box.hpp>
#include <engine/physics/Contact.hpp>
#include <engine/physics/Plane.hpp>
#include <engine/physics/Sphere.hpp>

namespace engine::physics {

class Collisions {
 public:
  static bool collides(const Plane& plane, Sphere sphere);
  static void handleElasticCollision(const Plane& plane, Sphere& sphere);
  static float distanceSquared(const Plane& plane, Sphere sphere);

  static bool computeContact(const Box& box, const Sphere& sphere,
                             Contact& out);
  static void resolveCollision(Box& box, Sphere& sphere, Contact contact,
                               float coefficient_of_restitution);
  static void resolveElasticCollision(Box& box, Sphere& sphere,
                                      Contact contact);

  static bool computeContact(const Box& boxA, const Box& boxB, Contact& out);

 private:
  static std::array<glm::vec3, 3> getAxesFromQuaternion(glm::quat q);
  static float calculate_overlap(glm::vec3 axis, const Box& boxA,
                                 const Box& boxB,
                                 const std::array<glm::vec3, 3> axesA,
                                 const std::array<glm::vec3, 3> axesB);
};

}  // namespace engine::physics
