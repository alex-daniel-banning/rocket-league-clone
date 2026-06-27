#pragma once

#include "engine/physics/box.hpp"

namespace engine {

struct CarInput {
  float throttle = 0.0f;
  float steering = 0.0f;
};

struct CarParams {
  float engine_power = 500.0f;
  float cornering_stiffness = 1.0f;
};

// Gameplay controller for a single car. Does not own a body; it references its
// chassis Box by id (the body is owned by Match / the physics layer) and writes
// driving forces into that body each tick.
class Car {
 public:
  explicit Car(int chassis_id, CarParams params = {}) : chassis_id_(chassis_id), params_(params) {}

  void SetInput(CarInput input) { input_ = input; }
  int chassis_id() const { return chassis_id_; }

  // Reads chassis state and writes the drive/steering force + torque into the
  // chassis force/torque accumulators (zeroing them first).
  void AccumulateDrivingForces(physics::Box& chassis) const;

 private:
  int chassis_id_;
  CarInput input_{};
  CarParams params_;
};

}  // namespace engine
