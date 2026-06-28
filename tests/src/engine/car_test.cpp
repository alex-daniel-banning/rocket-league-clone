#include "engine/car.hpp"

#include <cmath>
#include <glm/glm.hpp>
#include <gtest/gtest.h>

#include "engine/physics/box.hpp"
#include "engine/physics/box_builder.hpp"

using engine::Car;
using engine::CarParams;
using engine::physics::Box;
using engine::physics::BoxBuilder;

namespace {
constexpr float k_gravity = 9.8f;

// Identity-oriented chassis: local forward = +Z, right = +X, up = +Y.
Box MakeChassis(glm::vec3 velocity, glm::vec3 angular = glm::vec3(0.0f), float mass = 10.0f) {
  return BoxBuilder()
      .Size(glm::vec3(2.0f, 1.5f, 5.0f))
      .Position(glm::vec3(0.0f))
      .Mass(mass)
      .Velocity(velocity)
      .AngularVelocity(angular)
      .Id(1)
      .Build();
}

// Friction-circle cap for one wheel: mu * (static load per wheel).
float MaxWheelForce(const Box& b, const CarParams& p) { return p.tire_mu * b.mass * k_gravity * 0.5f; }
}  // namespace

// A car sliding straight sideways gets a lateral force opposing the slide, with
// (near-)zero net yaw torque -- the front and rear tires are symmetric here, so
// it's pure deceleration, not rotation.
TEST(CarTire, SidewaysVelocityProducesOpposingLateralForce) {
  Box chassis = MakeChassis(glm::vec3(5.0f, 0.0f, 0.0f));  // moving along +X (car right)
  Car car(chassis.GetId());
  car.AccumulateDrivingForces(chassis);

  EXPECT_LT(chassis.force_accumulator.x, 0.0f);  // opposes the +X slide
  EXPECT_NEAR(chassis.force_accumulator.y, 0.0f, 1e-3f);
  EXPECT_NEAR(chassis.force_accumulator.z, 0.0f, 1e-3f);
  EXPECT_NEAR(chassis.torque_accumulator.y, 0.0f, 1e-3f);  // symmetric front/rear -> no yaw
}

// Integrating the lateral tire force over time bleeds off sideways velocity --
// the car stops sliding instead of carrying its momentum sideways forever.
TEST(CarTire, LateralVelocityDecaysOverTime) {
  Box chassis = MakeChassis(glm::vec3(5.0f, 0.0f, 0.0f));
  Car car(chassis.GetId());
  const float dt = 1.0f / 120.0f;
  const float initial = std::abs(chassis.velocity.x);

  for (int i = 0; i < 30; i++) {
    car.AccumulateDrivingForces(chassis);
    chassis.velocity += chassis.force_accumulator * chassis.mass_inv * dt;
    chassis.force_accumulator = glm::vec3(0.0f);
    chassis.torque_accumulator = glm::vec3(0.0f);
  }

  EXPECT_LT(std::abs(chassis.velocity.x), initial);
}

// Steering yaws the car toward the steer direction. Input convention is
// steering = +1 -> turn right; in our right-handed +Y-up frame a right turn is
// a *negative* yaw torque about up (positive yaw is counter-clockwise / left).
TEST(CarTire, SteeringYawsTowardSteerDirection) {
  Box right_turn = MakeChassis(glm::vec3(0.0f, 0.0f, 8.0f));  // driving forward (+Z)
  Car car_right(right_turn.GetId());
  car_right.SetInput({0.0f, 1.0f});  // steer right
  car_right.AccumulateDrivingForces(right_turn);
  EXPECT_LT(right_turn.torque_accumulator.y, 0.0f);

  Box left_turn = MakeChassis(glm::vec3(0.0f, 0.0f, 8.0f));
  Car car_left(left_turn.GetId());
  car_left.SetInput({0.0f, -1.0f});  // steer left
  car_left.AccumulateDrivingForces(left_turn);
  EXPECT_GT(left_turn.torque_accumulator.y, 0.0f);
}

// No tire force may exceed the friction circle: at extreme slip both wheels
// saturate at exactly mu * load each, never more.
TEST(CarTire, LateralForceCappedByFrictionCircle) {
  Box chassis = MakeChassis(glm::vec3(50.0f, 0.0f, 0.0f));  // huge sideways slip -> saturated
  CarParams params;
  Car car(chassis.GetId(), params);
  car.AccumulateDrivingForces(chassis);

  const float max_total = 2.0f * MaxWheelForce(chassis, params);
  const float lateral_mag = std::abs(chassis.force_accumulator.x);
  EXPECT_LE(lateral_mag, max_total + 1e-2f);
  EXPECT_NEAR(lateral_mag, max_total, 1e-2f);  // both wheels at the cap
}

// Throttle produces a forward drive force with no yaw (it acts along the
// heading, through the centerline).
TEST(CarTire, ThrottleProducesForwardForceNoYaw) {
  Box chassis = MakeChassis(glm::vec3(0.0f));  // at rest -> no slip, no tire force
  CarParams params;
  Car car(chassis.GetId(), params);
  car.SetInput({1.0f, 0.0f});
  car.AccumulateDrivingForces(chassis);

  EXPECT_NEAR(chassis.force_accumulator.z, params.engine_power, 1e-3f);  // forward = +Z
  EXPECT_NEAR(chassis.force_accumulator.x, 0.0f, 1e-3f);
  EXPECT_NEAR(chassis.torque_accumulator.y, 0.0f, 1e-3f);
}
