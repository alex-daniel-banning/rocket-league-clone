# Where I'm at (2026-06-27)
Branch `car-control`, up to date with origin. Last commit:
`b6916ff car/box separation refactor`. Working tree clean.

Force-based driving works (throttle + steering). This session was the cleanup
checkpoint on top of "basic driving added": extracted a first-class `Car` and
de-godded `Match`. Full review + remaining plan in
`notes/code-review-car-control.md` (esp. Appendix A).

### Done this session
- `Car` extracted as an **id-referencing controller** (data-oriented "approach B"):
  `Car` holds `chassis_id_` + `CarInput` + `CarParams`; it does NOT own a Box.
  `boxes_` still owns the chassis; `bodies_` is still the solver's view of all bodies.
- Driving-force logic moved out of `Match` into `Car::AccumulateDrivingForces`
  (`src/engine/car.{hpp,cpp}`). Tuning constants now live in `CarParams`.
- `Builder::WithCar` + `car_box_indices_` wire chassis boxes to cars after id
  assignment (`BuildCars`). `Match::ResolveChassis(car)` maps id -> Box&.
- Killed the `boxes_[0]` magic: `IntegrateForces` now integrates ALL boxes.
- `WorldInverseInertia` de-duplicated into `include/engine/physics/inertia.hpp`.

### Next up (in order)
1. [ ] FINE-TUNE DRIVING in `./build/bin/car_control_demo` — get the feel right now
       that the refactor is in. Tune `CarParams` (engine_power, cornering_stiffness)
       and the damping. Watch how throttle accelerates and steering yaws.
       - We likely need to ADD FRICTION: the car probably slides/understeers without
         real lateral grip. Options to explore: tune the car-vs-ground contact
         friction (Box.friction + solver), and/or a tire-friction model. Decide which.
2. [ ] Lock the camera to the car in `car_control_demo` (follow the car's position;
       decide chase vs fixed-offset and whether to follow yaw). Currently it's the
       free fly camera, which makes driving hard to evaluate.
3. [ ] Quick review-cleanups (low risk, see code-review §3/§4):
       - Extract the duplicated orientation-integration + damping blocks in `Tick`
         (ball vs each box) into one helper.
       - Collapse `GenerateContactConstraints` (6 copies of ComputeContact +
         GenerateFromContact) behind a `try_pair` lambda.
4. [ ] FEATURE: 4-point raycast ground detection — cast a ray at each wheel point,
       only apply throttle where there's ground contact.
       -> `notes/raycast-ground-detection.md` (slab-method ray-vs-OBB, throttle gate,
          gotchas, tests). Context: `notes/car-control-and-suspension.md`.

### Deferred (not now)
- `ApplyGravity` -> force-accumulator unification (review §6); two force paths today.
- Multi-car input routing: `SetCarInput` hardcodes `cars_[0]` (TODO is in the code).
  Generalize the signature only when a 2nd car exists.

### Design decisions still in force (relevant to suspension)
- Inputs normalized: throttle/steering in [-1,1]; `CarParams` maps to N / N·m.
- Steering = yaw torque from a lateral force at the front (in `Car`).
- `ApplyForceAtPoint` (cross(p-pos, F)) is the tool for PER-WHEEL SUSPENSION next.
- Torque integration: ω += (R·I⁻¹_body·Rᵀ)·τ·dt via `WorldInverseInertia`.
  Gyroscopic term ω×(Iω) intentionally omitted.
