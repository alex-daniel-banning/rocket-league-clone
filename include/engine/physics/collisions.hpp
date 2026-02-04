#pragma once

#include <engine/physics/Contact.hpp>
#include <engine/physics/Sphere.hpp>
#include <engine/physics/box.hpp>
#include <engine/physics/collision_type.hpp>
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
  static void ResolveCollision(Box& box_a, Box& box_b, const Contact contact,
                               float coefficient_of_restitution);

 private:
  static std::array<glm::vec3, 3> GetAxesFromQuaternion(glm::quat q);
  static float CalculateOverlap(glm::vec3 axis, const Box& box_a,
                                const Box& box_b,
                                const std::array<glm::vec3, 3> axes_a,
                                const std::array<glm::vec3, 3> axes_b);
  static CollisionType DetermineCollisionType(
      const glm::vec3 penetration_axis, const std::array<glm::vec3, 3>& axes_a,
      const std::array<glm::vec3, 3>& axes_b);
  static std::vector<glm::vec3> ClipFaceFace(
      const Box& box_a, const Box& box_b,
      glm::vec3 penetration_axis,  // TODO make const?
      const std::array<glm::vec3, 3>& axes_a,
      const std::array<glm::vec3, 3>& axes_b);
  static std::vector<glm::vec3> ClipPolygonAgainstPlane(
      const std::vector<glm::vec3>& polygon, glm::vec3 plane_point,
      glm::vec3 plane_normal, const float epsilon = 0.001f);
  static std::vector<glm::vec3> ClipEdgeToFace(
      const Box& box_inc, const Box& box_ref, glm::vec3 penetration_axis,
      const std::array<glm::vec3, 3>& axes_inc,
      const std::array<glm::vec3, 3>& axes_ref);
  // static int CalculateNumberOfSymmetricalEdges(
  //     const glm::vec3 penetration_axis, const std::array<glm::vec3, 3>&
  //     axes_a, const std::array<glm::vec3, 3>& axes_b);
  static std::vector<glm::vec3> ClipEdgeEdge(
      const Box& box_a, const Box& box_b, glm::vec3 penetration_axis,
      const std::array<glm::vec3, 3>& axes_a,
      const std::array<glm::vec3, 3>& axes_b);
  static std::vector<glm::vec3> ClipCornerToFace(  // TODO, make these anonymous
      const Box& box_inc, const Box& box_ref, glm::vec3 penetration_axis,
      const std::array<glm::vec3, 3>& axes_inc,
      const std::array<glm::vec3, 3>& axes_ref);
};

}  // namespace engine::physics
