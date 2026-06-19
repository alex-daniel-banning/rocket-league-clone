## Where I'm at (2026-06-19)
Working Phase 3 (force-based control) in `match.cpp`. Throttle force + torque
integration wired into `IntegrateForces` / `AccumulateDrivingForces`.

CURRENTLY DEBUGGING: NaN in `force_accumulator` ({-nan, -nan, ~1e-14}).
- Root cause is NOT line 87 (`-100 * throttle`) — multiplication only propagates NaN.
- NaN is already in `direction = normalize(rotation * (0,0,1))` (line 84) -> the
  `rotation` quaternion itself is corrupted (NaN in x/y components).
- The {nan, nan, finite} pattern rules out a zero-length normalize (that'd be all 3 nan);
  it means rotation went bad in a PREVIOUS tick.
- Suspect: angular_velocity going NaN/blowing up, then poisoning the quaternion update
  in Tick (match.cpp:42-44). Next step: assert !isnan on angular_velocity after
  IntegrateForces and on rotation after the quat update; find the FIRST tick it fires.

Design decisions settled this session:
- Inputs normalized: throttle/steering in [-1,1]; engine maps to N / N·m via constants.
- Steering = add yaw torque DIRECTLY (torque += steering*k*speed*up), not cross(r,F).
- cross(p-pos, F) / ApplyForceAtPoint is for SUSPENSION (per-wheel), not steering.
- Torque integration: ω += (R·I⁻¹_body·Rᵀ)·τ·dt; reuse WorldInverseInertia
  (constraint_solver.cpp:18). Gyroscopic term ω×(Iω) intentionally omitted.

---

[ ] Make it so there are 4 points on the car that cast a ray to check if there is a contact point where the wheels are, then apply throttle under that condition
    -> See notes/raycast-ground-detection.md for the approach (slab-method ray-vs-OBB, throttle gate, gotchas, tests)
