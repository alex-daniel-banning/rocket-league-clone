#pragma once

#include "engine/physics/box.hpp"

namespace engine {

struct CarInput {
  float throttle = 0.0f;
  float steering = 0.0f;
};

struct CarParams {
  // Forward drive force at full throttle (N), applied along the car's heading.
  float engine_power = 500.0f;

  // Real cornering stiffness: lateral tire force generated per radian of slip
  // angle (N/rad), in the linear region before the friction circle saturates.
  // At cornering_stiffness=1000 and the demo car's ~110 N per-wheel grip cap, a
  // tire saturates around 0.11 rad (~6 deg) of slip. See
  // notes/tire-friction-model.md.
  float cornering_stiffness = 1000.0f;

  // --- Tire model params (consumed from Phase 1 onward) ---
  // Maximum front-wheel steer angle at full steering input (radians).
  float max_steer_angle = 0.5f;
  // Tire-ground friction coefficient; caps each wheel's force at mu * load
  // (the friction circle).
  float tire_mu = 5.0f;
  // Wheel contact points sit at the chassis corners, inset from the half-extents
  // by these fractions along the car's local axes (Z = forward / wheelbase,
  // X = right / track). 1.0 places a wheel exactly at the half-extent edge.
  float wheel_base_frac = 0.8f;
  float wheel_track_frac = 0.9f;
};

// Gameplay controller for a single car. Does not own a body; it references its
// chassis Box by id (the body is owned by Match / the physics layer) and writes
// driving forces into that body each tick.
class Car {
 public:
  explicit Car(int chassis_id, CarParams params = {}) : chassis_id_(chassis_id), params_(params) {}

  void SetInput(CarInput input) { input_ = input; }
  int ChassisId() const { return chassis_id_; }

  // Reads chassis state and adds the drive/steering force + torque into the
  // chassis force/torque accumulators. Accumulators are cleared centrally in
  // Match::IntegrateForces after integration, so this only adds.
  void AccumulateDrivingForces(physics::Box& chassis) const;

 private:
  int chassis_id_;
  CarInput input_{};
  CarParams params_;
};

}  // namespace engine
