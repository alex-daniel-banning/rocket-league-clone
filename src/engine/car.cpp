#include "engine/car.hpp"

#include <algorithm>
#include <cmath>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

namespace engine {

namespace {
constexpr float k_gravity = 9.8f;
// Below this forward speed the slip angle is computed against a floored
// denominator, so a near-stationary wheel can't produce a huge (or undefined)
// slip angle. See notes/tire-friction-model.md "low-speed blowup".
constexpr float k_min_slip_speed = 1.0f;

// Accumulate a force applied at a world-space point: the linear force plus the
// torque it induces about the body's center of mass.
void ApplyForceAtPoint(physics::Box& body, glm::vec3 force, glm::vec3 point) {
  body.force_accumulator += force;
  body.torque_accumulator += glm::cross(point - body.position, force);
}

// Lateral (cornering) force a single tire generates from its slip angle, using a
// linear-then-saturating "brush" model: F = -clamp(C_alpha * alpha, +/- mu*N).
// The tire opposes its own sideways slip, which is what re-aligns the car's
// velocity with its heading (i.e. grip). `wheel_forward`/`wheel_right` are the
// tire's heading axes (steered, for the front wheel); `r` is the wheel position
// relative to the center of mass.
glm::vec3 TireLateralForce(const physics::Box& chassis, glm::vec3 r, glm::vec3 wheel_forward, glm::vec3 wheel_right,
                           float cornering_stiffness, float max_force) {
  glm::vec3 v_wheel = chassis.velocity + glm::cross(chassis.angular_velocity, r);
  float v_long = glm::dot(v_wheel, wheel_forward);
  float v_lat = glm::dot(v_wheel, wheel_right);
  float slip_angle = std::atan2(v_lat, std::max(std::abs(v_long), k_min_slip_speed));
  float f_lat = std::clamp(cornering_stiffness * slip_angle, -max_force, max_force);
  return -f_lat * wheel_right;
}
}  // namespace

void Car::AccumulateDrivingForces(physics::Box& chassis) const {
  const glm::vec3 forward = chassis.rotation * glm::vec3(0.0f, 0.0f, 1.0f);
  const glm::vec3 right = chassis.rotation * glm::vec3(1.0f, 0.0f, 0.0f);
  const glm::vec3 up = chassis.rotation * glm::vec3(0.0f, 1.0f, 0.0f);

  // Bicycle model: two wheels on the car's centerline, fore and aft of the
  // center of mass. Wheels sit at CoM height (local y = 0) so a horizontal tire
  // force produces pure yaw, no roll -- roll/suspension coupling comes later.
  // TODO(Phase 3): gate tire forces on per-wheel ground contact (raycast);
  // for now the car is assumed grounded.
  const float half_base = params_.wheel_base_frac * chassis.HalfExtents().z;
  const glm::vec3 r_front = forward * half_base;
  const glm::vec3 r_rear = -forward * half_base;

  // Static load per wheel (equal split). Dynamic per-wheel load (weight transfer
  // / suspension) is a later phase.
  const float load = chassis.mass * k_gravity * 0.5f;
  const float max_force = params_.tire_mu * load;

  // The front wheel's heading is steered about the chassis up axis; the rear
  // wheel stays aligned with the body. Input convention: steering = +1 means
  // steer right. In our right-handed, +Y-up frame a positive rotation about up
  // is counter-clockwise seen from above (a *left* turn), so a right turn needs
  // a negative steer angle -- hence the minus sign.
  const float steer_angle = -input_.steering * params_.max_steer_angle;
  const glm::quat steer = glm::angleAxis(steer_angle, up);
  const glm::vec3 front_forward = steer * forward;
  const glm::vec3 front_right = steer * right;

  // Lateral tire forces -- the grip that stops the car sliding through corners.
  glm::vec3 front_lat =
      TireLateralForce(chassis, r_front, front_forward, front_right, params_.cornering_stiffness, max_force);
  glm::vec3 rear_lat = TireLateralForce(chassis, r_rear, forward, right, params_.cornering_stiffness, max_force);
  ApplyForceAtPoint(chassis, front_lat, chassis.position + r_front);
  ApplyForceAtPoint(chassis, rear_lat, chassis.position + r_rear);

  // Drive force: forward at the rear (drive) wheel. Collinear with the heading,
  // so it adds no yaw. Longitudinal grip (the friction circle) is a later phase.
  glm::vec3 drive_force = params_.engine_power * input_.throttle * forward;
  ApplyForceAtPoint(chassis, drive_force, chassis.position + r_rear);
}

}  // namespace engine
