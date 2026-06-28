# Tire friction model

Status: planned. Pairs with `car-control-and-suspension.md` and
`raycast-ground-detection.md`. Replaces the current "steering = commanded
lateral force at the front" hack (`Car::AccumulateDrivingForces`, `car.cpp:17`)
with a slip-based tire model so the car stops sliding around in corners.

## Why we need this

The current slide happens because *nothing re-aligns velocity to heading*: we
yaw the chassis but momentum carries it straight. `cornering_stiffness` today is
a steering-authority gain (a misnomer), not grip — turning it up makes the slide
worse. The fix is a real tire model where wheels generate force from *slip*.

---

## Part 1 — The conceptual model

### Core idea: tires make force by slipping

A tire is not a frictionless rolling wheel. Its contact patch deforms and
generates force **in proportion to how much it's slipping** — the mismatch
between where the wheel points/rolls and where it's actually moving. No slip →
no force. This is the opposite of commanding a lateral force from steering input.

### Slip angle -> lateral (cornering) force

For one wheel, decompose its contact-point velocity into the wheel's own axes:

```
v_long = dot(v_wheel, wheel_forward)   // rolling speed
v_lat  = dot(v_wheel, wheel_right)     // sideways sliding speed
```

Slip angle (heading vs travel):

```
alpha = atan2(v_lat, |v_long|)
```

Lateral force opposes the slip, linear at small angles:

```
F_lat = -C_alpha * alpha
```

`C_alpha` is the **real cornering stiffness**: lateral force per radian of slip
(N/rad). This is the proper meaning of the parameter we currently misuse.

### The tire curve: linear, then saturation

`F_lat` can't grow forever — the patch can only transmit so much before it lets
go. The cap is the friction limit:

```
|F_lat| <= mu * N        (N = normal load on the wheel)
```

So the real curve is linear (slope C_alpha) then flattens/saturates at mu*N.
Implement the simplest faithful version first — **brush model simplified to
linear-then-clamp**:

```
F_lat = -clamp(C_alpha * alpha, -mu*N, +mu*N)
```

The industry-standard empirical curve is **Pacejka's "Magic Formula"** (a fitted
curve with a distinct peak and falloff). That's a polish item (stage 4), not the
starting point.

### Slip ratio -> longitudinal (drive/brake) force

The longitudinal analog: a driven wheel spins slightly faster than the ground.
That **slip ratio** generates forward force the same way (linear then saturating
at mu*N). We don't model wheel rotational speed yet, so keep throttle as a
commanded forward force and just subject it to the same friction cap for now.

### Friction circle (combined slip) — the payoff

Longitudinal and lateral forces share one budget:

```
sqrt(F_long^2 + F_lat^2) <= mu * N
```

A tire using all its grip to accelerate has none left to corner — flooring it
mid-corner understeers; trail-braking rotates the car. Lots of emergent behavior
falls out of this one rule.

### Load (N)

Grip scales with normal force. Without suspension, approximate static load
`N ~= m*g / num_wheels`. Later, suspension + **load transfer** (weight shifts
forward under braking, to outside wheels when cornering) makes N dynamic per
wheel — that's the car's weight-feel. Stage 4.

### Why this kills the slide

The moment the car's velocity points sideways relative to a wheel, that wheel
develops a slip angle and pushes back laterally. The **rear** wheels act as a
weathervane, dragging the velocity vector back in line with the body. Steering
gives the **front** wheels a steer angle so they develop slip and pull the nose
around; the difference between front and rear grip is over/understeer.

---

## Part 2 — How this maps to the codebase

Integration point: **`Car::AccumulateDrivingForces(Box& chassis)`** (`car.cpp:17`)
already runs each substep before `IntegrateForces` and writes to
`force_accumulator`/`torque_accumulator` via `ApplyForceAtPoint`. Rewrite the
*inside* into a per-wheel loop. Tick order (`match.cpp:13`) does not change.

Three decisions (with the chosen answer):

1. **Double-counting.** The chassis box has `friction = 0.5` and rests on the
   ground via box-box contact, so the contact solver's isotropic Coulomb
   friction already provides grip (`constraint_solver.cpp:219`). Adding tire
   forces on top makes two systems fight.
   -> **Decision:** set chassis box friction ~0; the tire model owns all
   tangential grip. Contact solver keeps doing the vertical normal force.

2. **Low-speed blowup.** `alpha = atan2(v_lat, |v_long|)` is unstable as
   `|v_long| -> 0`, and with friction zeroed a parked car would slide if bumped.
   -> **Decision:** floor `|v_long|` (e.g. 0.5-1.0 m/s) so slip angle tapers
   smoothly; optionally blend to a static-grip term at crawl speed. Build into
   phase 1.

3. **Ground gating.** Tire forces only exist when the wheel is grounded — this is
   the planned 4-point raycast (`raycast-ground-detection.md`).
   -> **Decision:** design the per-wheel helper to take a `grounded`/`load`
   input. For now assume grounded (no jumping yet) or a simple height check; the
   4 tire points and the 4 raycast points are the same four — do them aligned.

---

## Part 3 — Phased implementation plan

### Phase 0 — Prep (params + geometry)
- Rework `CarParams` (`car.hpp:12`): keep `engine_power`; replace
  `cornering_stiffness` with real tire params — `cornering_stiffness` (N/rad),
  `max_steer_angle` (rad), `tire_mu`. Add wheel layout (4 local offsets from
  chassis center, derived from `HalfExtents`, or explicit wheelbase/track).
- Set chassis box friction ~0 at the car build site (`Builder::WithCar`).
- Verify: car still drives; now slides *more* freely (confirms friction zeroed).

### Phase 1 — Lateral tire force (the core fix), bicycle model
Start with 2 wheels (one front, one rear, on the centerline) to isolate the
concept.
- Per-wheel helper: wheel world position; contact velocity
  `v = chassis.velocity + cross(angular_velocity, r)`; decompose long/lat;
  `alpha` with low-speed clamp; `F_lat = -clamp(C_alpha*alpha, +/- mu*N)`;
  `ApplyForceAtPoint`.
- `N = m*g/2` static. Front wheel forward rotated by `steering*max_steer_angle`.
- Keep `engine_power*throttle` as a forward force at the rear wheel for now.
- Verify in `car_control_demo`: lateral velocity with no steering straightens
  out instead of sliding; steering traces a stable circle.
- Tests (GoogleTest + BoxBuilder): (a) pure sideways velocity develops an
  opposing lateral force that decays v_lat; (b) steady steering reaches a stable
  yaw rate; (c) force never exceeds mu*N.

### Phase 2 — Friction circle + longitudinal coupling
- Move drive/brake into the per-wheel budget; compute desired (F_long, F_lat),
  scale to `sqrt(F_long^2 + F_lat^2) <= mu*N`.
- Verify: flooring throttle mid-corner reduces cornering grip. Test: combined
  magnitude capped at mu*N.

### Phase 3 — Generalize to 4 wheels
- Same helper, four corner offsets; front two steerable. Wire `grounded`/`load`
  so the 4-point raycast drops in.

### Phase 4 — Polish (optional, later)
- Pacejka magic-formula curve; dynamic load transfer (needs suspension);
  longitudinal slip-ratio model (needs wheel angular velocity).

---

## Part 4 — Risks & tuning notes

- **Stiffness/integration instability:** `C_alpha` is a stiff spring-like term;
  too high with explicit Euler at `fixed_dt` causes shimmy. Substeps help; clamp
  slip angle, start C_alpha modest, raise until it twitches, back off.
- **Tuning order:** `mu` (peak grip) and `C_alpha` (how fast grip builds) in
  phase 1; `max_steer_angle` sets turn radius; front-vs-rear C_alpha/mu split
  (phase 3) sets over/understeer balance.
- **Keep the old path** available until phase 1 feels right for easy A/B.
