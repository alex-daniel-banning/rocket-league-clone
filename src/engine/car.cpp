#include "engine/car.hpp"

#include <glm/geometric.hpp>

namespace engine {

namespace {
// Accumulate a force applied at a world-space point: the linear force plus the
// torque it induces about the body's center of mass.
void ApplyForceAtPoint(physics::Box& body, glm::vec3 force, glm::vec3 point) {
  body.force_accumulator += force;
  body.torque_accumulator += glm::cross(point - body.position, force);
}
}  // namespace

// TODO figure out reference units?
void Car::AccumulateDrivingForces(physics::Box& chassis) const {
  chassis.force_accumulator = glm::vec3();
  chassis.torque_accumulator = glm::vec3();

  glm::vec3 car_forward = chassis.rotation * glm::vec3(0.0f, 0.0f, 1.0f);
  glm::vec3 right = chassis.rotation * glm::vec3(1.0f, 0.0f, 0.0f);

  // --- Drive force: forward, at the center of mass (no torque) ---
  glm::vec3 drive_force = params_.engine_power * input_.throttle * glm::normalize(car_forward);
  ApplyForceAtPoint(chassis, drive_force, chassis.position);

  // --- Steering force: lateral, at the front wheels (produces yaw torque) ---
  float forward_speed = glm::dot(car_forward, chassis.velocity);
  glm::vec3 lateral_force = -input_.steering * forward_speed * params_.cornering_stiffness * right;
  glm::vec3 steering_point = chassis.position + (chassis.HalfExtents().z * car_forward);
  ApplyForceAtPoint(chassis, lateral_force, steering_point);
}

}  // namespace engine
