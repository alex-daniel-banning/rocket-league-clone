# Controllable Car + Raycasted Suspension

A roadmap for getting a drivable car that interacts with the world, building toward
raycasted suspension. Written as a learning guide, not a checklist — it explains the
concepts and points at the code to study so the implementation is yours to write.

## The big picture

Today a "car" is just a `Box` in `Match::boxes_`: it falls under gravity, collides with
the ground/walls/ball via the existing box-box solver, and is drawn as a colored box.
There's no way to control it, no notion of applying a force, and no way to cast a ray
into the world.

The through-line of this plan: **everything is driven by forces.** Add one small piece
of machinery — the ability to apply a force at a point on a body — and both *driving*
(Phase 3) and *suspension* (Phase 4) fall out of it. That's the reason for the chosen
ordering: each phase builds the foundation the next one reuses.

Decisions already made: force-based driving (not arcade/kinematic), throttle + steering
only for now (jump/aerial later), and render the real `car.obj` with a wireframe hitbox.

## Phase 1 — Render the car properly

**Goal:** see `resources/car/car.obj` drawn at a car's position/orientation.

**Concepts to get comfortable with:**
- How a model's transform is assembled from position + rotation (quaternion) + scale.
  Study `Renderer::MakeModelMatrix` and `DrawBox`/`DrawModel` in
  `src/engine/render/renderer.cpp` to see the translate → rotate → scale order.
- Asset loading: `engine::render::Model` (Assimp) + `PathManager::GlobalAsset("car/car.obj")`.

**Where to start:** copy the structure of `src/demos/perf/perf_demo.cpp` into a new
`src/demos/car/car_control_demo.cpp` and add it to `src/demos/CMakeLists.txt`. Render the
model at a box's `position`/`rotation`.

**Expect to tune:** the .obj's native scale and "forward" axis almost certainly won't
match our world units, so you'll want a `model_scale` and a `model_rotation_offset` to
line it up. This is trial-and-error — run it and adjust.

**Done when:** the car model shows up, oriented sensibly, sitting on the ground.

## Phase 2 — Hitbox

**Goal:** a collision shape that matches the visible car.

**Key idea:** the physics `Box` *is* the hitbox — there's no separate collision type to
build. The work is (a) choosing box dimensions that match the model's bounding volume
(Octane-ish proportions, something like `(1.2, 0.4, 2.0)`, tuned by eye) and (b) drawing
it so you can see it. `DrawBoxWireframe` already exists — overlay it on the model.

**Worth understanding:** why a single box is a fine hitbox for an arcade car (the real
game uses a simple hull too), and how the model and hitbox are *different representations
of the same transform* — they share `position`/`rotation` but have independent size/scale.

**Done when:** the wireframe encloses the model and the car rests on the ground.

## Phase 3 — Force-based control (throttle + steering)

This is the foundational phase. The new machinery here is small but it's what makes
everything afterward possible.

**Concept 1 — force/torque accumulators.** Real integrators gather all the forces acting
on a body during a step, then apply them at once. Add two mutable fields to `Box`:
`force_accumulator` and `torque_accumulator`. Each substep you'll integrate them into
velocity and angular velocity, then zero them. Compare this to how `ApplyGravity` in
`src/engine/match.cpp` currently nudges velocity directly — you're generalizing that idea.

**Concept 2 — a force applied off-center creates rotation.** This is the crux. A force
`F` applied at world point `p` produces a torque `τ = (p − center) × F`. So
`ApplyForceAtPoint(F, p)` does two things: `force_accumulator += F` and
`torque_accumulator += cross(p − position, F)`. Internalize this — it's *the* idea that
makes both steering and suspension work.

**Concept 3 — integrating torque needs world-space inverse inertia.** Angular update is
`angular_velocity += I⁻¹_world · torque · dt`, where `I⁻¹_world = R · I⁻¹_body · Rᵀ`.
Good news: this exact computation already exists as `WorldInverseInertia` (anon namespace
in `src/engine/physics/constraint_solver.cpp`). Pulling it onto `Box` as a method so both
the solver and your new integration share it is a nice refactor — and a good exercise in
spotting reuse.

**Driving model (simple to start):**
- Forward axis from the car's orientation (`rotation * (0,0,-1)`, projected flat so you
  don't drive into the ground). Throttle → a forward force.
- Steering → a yaw torque about the up axis. (A nice realism detail to experiment with
  later: scale steering by forward speed so a parked car doesn't spin in place.)
- These have tunable constants — keep them named and grouped so they're easy to feel out.

**Plumbing to figure out:** input only reaches the camera today (`ProcessInput` in
`src/demos/demo_common.hpp`), and `Match` only exposes a const `GetBoxes()`. You'll need
a path from keys → the player car. Think about *where* driving forces should be applied:
`Tick` runs several fixed substeps per frame, so forces belong inside that loop, which
suggests passing an input *state* into `Match` rather than mutating a box from the demo.
A small `CarInput { throttle, steer }`, a way to mark which box is the player car, and a
setter is enough. (Designing this interface yourself is good practice — there are a few
reasonable shapes.)

**Done when:** W/S accelerate/reverse and A/D steer the car around on the ground.

## Phase 4 — Raycasted suspension

Now the payoff. See `raycasted-suspension.md` for the full design; the essentials:

**Concept 1 — raycasting.** You need to ask "from this point, shooting in this
direction, where do I hit the ground?" No raycast exists yet, so you'll build a
ray-vs-box (OBB) test. The standard technique is the **slab method**: transform the ray
into the box's local space (using its rotation and `half_extents_`), intersect against
the three axis-aligned slabs, and take the overlapping interval. Returning hit distance,
point, and normal is enough. This is a great self-contained thing to write and unit-test.

**Concept 2 — a wheel is a spring + damper, not a collision.** For each of four mount
points under the car, cast a short ray down. If it hits within `rest_length`, the
suspension is compressed by `rest_length − hit_distance`. Apply an upward force:
`spring = k · compression` plus a damper term `−c · (velocity of the mount along the ray)`
that bleeds out oscillation. Because you apply it with `ApplyForceAtPoint` at the wheel,
the four forces *automatically* produce pitch/roll — no special-casing. That's the elegance.

**Concept 3 — critical damping.** `c = 2·√(k·m)` is the boundary between bouncy
(underdamped) and sluggish (overdamped). Start near it and tune. Understanding why is
worth a few minutes with the spring equation.

**The one integration subtlety — keep ground collision on, as a backstop.** Suspension
and the contact solver coexist; suspension does *not* replace car-ground collision. Tune
the ride height (`rest_length` vs. `k`) so that at equilibrium the hitbox *floats just
above* the ground — at rest there's no penetration, so the solver naturally generates no
car-ground contacts and the springs alone hold the car up. Ground collision then only
wakes up in the extreme cases the springs can't cover: bottoming out on a hard landing,
or the car resting on its roof/side (where the downward rays miss the ground entirely
and, without collision, the car would fall through the floor). One detail that makes this
robust: cast each wheel ray from a fixed point on the body *above* the wheel travel, not
from the contact point — that keeps the ray origin above the surface under heavy
compression so it never loses the ground right when the force is needed.

**Done when:** the car hovers at rest length and settles smoothly, tips over bumps/the
ball, and free-falls when it drives off an edge (rays miss → zero force).

## Checking your work

- The ray-vs-box test is pure math with clear right answers — a good candidate for a
  GoogleTest file (follow the patterns in `tests/src/engine/physics/`): hits vs misses on
  an axis-aligned *and* a rotated box, correct distance/normal, ray starting inside,
  parallel-miss. `ApplyForceAtPoint` is also easily testable (off-center force → expected
  torque; one integration step → expected velocities).
- Everything else is felt, not asserted — build and drive the demo.

## Deferred (later milestones)

Jump, aerial pitch/yaw/roll, flip/dodge recovery, driving on walls and the ceiling
(needs surface-normal-aware raycasting, not just a downward ray), speed-dependent
friction, boost. Ball rolling friction is a separate, unrelated backlog item.
