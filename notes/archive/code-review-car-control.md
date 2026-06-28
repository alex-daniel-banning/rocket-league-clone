# Code Review — `car-control` → `master` (cleanup checkpoint)

**Scope reviewed:** branch diff `master...car-control` (7 commits, ~530 lines).
**Focus:** general design & code organization, per request. Not a pre-merge
gate — this is a checkpoint while more features are still landing.
**Date:** 2026-06-27

Overall the structure is healthy: `include/` mirrors `src/`, namespaces map to
directories, Builder patterns are consistent, and the new `inertia.hpp` extraction
is the right move. The notes below are about keeping `Match` from sprawling as the
car/gameplay layer grows. Nothing here blocks continued feature work.

---

## 1. Biggest organizational theme: `Match` is becoming a god class

`Match` currently owns at least five distinct responsibilities:

1. **Simulation orchestration** — the fixed-timestep loop in `Tick`.
2. **Generic rigid-body integration** — `ApplyGravity`, `IntegrateForces`,
   `ApplyForceAtPoint`, orientation integration.
3. **Car gameplay model** — `AccumulateDrivingForces` (engine power, cornering
   stiffness, steering torque). This is *vehicle design*, not physics.
4. **Collision broad-phase** — `GenerateContactConstraints` enumerates every body
   pair by hand.
5. **Input state** — holds `car_input_` (with your own `// TODO belongs here?`).

For a cleanup checkpoint, the highest-leverage move is to pull the **car/vehicle
concept out of `Match`**. A `Car` (or `VehicleController`) that owns its input and
its driving model would:
- remove the `boxes_[0]`-is-the-car assumption (see §2),
- give `CarInput` an obvious home,
- let `Match` shrink toward "owns bodies, steps the world, resolves contacts."

You don't need this today, but it's the refactor the rest of these notes keep
pointing back to. Worth doing before suspension/raycast wheels land, since those
add *more* per-car logic.

## 2. `boxes_[0]` magic indexing + asymmetric integration (correctness smell)

`AccumulateDrivingForces` (`match.cpp:82`) and `IntegrateForces` (`match.cpp:111`)
both hard-code `boxes_[0]` as "the car." Two problems:

- **No explicit car entity** — index 0 is an invisible contract between these two
  methods and the demo's box ordering.
- **Forces are accumulated for all boxes but only integrated for box 0.**
  `ApplyForceAtPoint` is called on whatever box is passed, but `IntegrateForces`
  only ever applies the accumulator on `boxes_[0]`. Any other box that received a
  force would silently never have it integrated. Today there's one box so it's
  latent, but it's a real trap. Either integrate all boxes in a loop, or make the
  single-car assumption explicit via a typed `Car` member rather than `boxes_[0]`.

This is the same root cause as §1 — there's no first-class "car."

## 3. Duplicated integration & damping in `Tick` (easy win)

`Tick` integrates orientation with an identical copy-pasted block for the ball
(`match.cpp:32-35`) and for each car (`match.cpp:38-41`), and again duplicates the
damping logic (`:49-55`). Extract one helper, e.g.:

```cpp
void IntegrateOrientation(glm::quat& rot, const glm::vec3& w, float dt);
```

…or a small free function operating on a body. This is the lowest-risk cleanup in
the diff and removes the most duplication. (The ball/car asymmetry — ball gets
linear damping only, cars get linear + angular — is worth a comment if intentional.)

## 4. `GenerateContactConstraints` — six copies of the same block

`match.cpp:121-187` repeats the `ComputeContact(...) -> GenerateFromContact(...)`
pattern six times (ball/wall, ball/ground, ball/car, car/car, car/wall, car/ground)
with only the operands changing. Collapse with a local helper:

```cpp
auto try_pair = [&](const auto& a, const auto& b) {
  physics::Contact contact;
  if (physics::collisions::ComputeContact(a, b, contact))
    physics::ConstraintSolver::GenerateFromContact(
        contact, bodies_, dt, constraints.normal, constraints.friction,
        restitution, baumgarte);
};
```

This is also the function your memory flags for the upcoming broad-phase / spatial
partitioning work — shrinking it now makes that swap cleaner. The hand-rolled O(n²)
nested loop is exactly the seam to isolate.

## 5. Dead / stray code to sweep

- `match.cpp:58` — `physics::Box& car = boxes_[0];` at the end of the `while` loop
  body. Unused, reads as a leftover.
- `match.cpp:115` — `glm::vec3 vel = acceleration * fixed_dt;` never used.
- `match.cpp:86` and `:90` compute the forward vector twice (`car_forward`, then
  `direction = normalize(rotation * (0,0,1))`); `car_forward` is already that
  direction (unit, since rotation is normalized).
- Demo has substantial commented-out rendering (`car_control_demo.cpp:89, 167,
  188-189`) and an unused `name_to_color` map (`:26`). Fine for a WIP demo, but
  worth a pass before this branch settles.

## 6. Two parallel force paths (already half-flagged)

Gravity is applied directly to velocity (`ApplyGravity`, with your
`// TODO convert to force accumulator paradigm`), while driving uses the
accumulator. So there are two integration paths and the accumulators are zeroed in
`AccumulateDrivingForces` for box 0 only. Unifying on the accumulator (gravity as a
force) would make `IntegrateForces` the single integration point and remove the
ordering fragility. Already on your radar — noting it for completeness.

## 7. Magic numbers scattered as locals

`car_engine_power_temp = 500` (`:92`), cornering stiffness `k = 1.0` (`:98`),
damping `0.98` (`:45-46`), `baumgarte = 0.02`, `restitution = 0.3` (`:124-125`),
`solver_iterations = 10` (`:29`). For a tuning-heavy project these will move a lot.
Consider a `SimParams` / `CarParams` struct so they're named, grouped, and
adjustable without hunting through the loop. Low priority, but it pays off fast once
you start tuning feel.

## 8. Minor

- **`Box` gaining `force_accumulator` / `torque_accumulator`** (`box.hpp:23-24`) —
  good fit for the data-oriented design. Just note `Box` now carries several
  distinct state categories (immutable props, sim state, split-impulse pseudo-vel,
  force accumulators); a comment grouping them would help future readers.
- **Gizmo comment mismatch** — `renderer.cpp` says "top-**left**" but the math and
  the header comment both say top-**right**. Trivial, but fix the comment.
- **Input handling in the demo** uses file-scope mutable globals (`throttle`,
  `steering`). Acceptable for a demo; just don't let that pattern migrate into
  engine code.

---

## Suggested order for a cleanup pass

Cheap, high-value, low-risk first:

1. Delete dead code (§5).
2. Extract orientation-integration + damping helper (§3).
3. Collapse `GenerateContactConstraints` with a `try_pair` helper (§4).
4. Fix the gizmo comment (§8).

Then the structural work, when you're ready (likely alongside suspension/raycast):

5. Introduce a first-class `Car` entity owning `CarInput` + driving model; drop
   `boxes_[0]` indexing and fix the all-boxes integration gap (§1, §2, §6).
6. Pull tuning constants into a params struct (§7).

---

## Appendix A: Decomposing `Match` (concrete plan)

Expands on §1. The goal is **not** an elaborate layer cake — for a project this
size that's its own trap. Make two targeted extractions and let `Match` stay the
orchestrator.

### Target: `Match` as composition root + step scheduler

`Match` keeps owning the bodies, the cars, and the solver, and its `Tick` reads as
a sequence of named phases. It stops *implementing* integration, driving, and
pair-enumeration inline — those become things it *calls*. Three responsibilities
peel off:

### 1. A `Car` entity (highest value — also fixes the §2 bug)

Owns its input and drive model, bound to a chassis body by **id, not reference**
(a `Box&` is fragile against `boxes_` reallocation — the same reason the existing
pointer map assumes `boxes_` is stable):

```cpp
namespace engine {
struct CarParams {           // §7 tuning constants land here
  float engine_power = 500.0f;
  float cornering_stiffness = 1.0f;
};

class Car {
 public:
  Car(int chassis_id, CarParams params) : chassis_id_(chassis_id), params_(params) {}
  void SetInput(CarInput in) { input_ = in; }
  int chassis_id() const { return chassis_id_; }

  // Reads chassis state, writes into its force/torque accumulators.
  void AccumulateDrivingForces(physics::Box& chassis) const;

 private:
  int chassis_id_;
  CarInput input_{};
  CarParams params_;
};
}
```

`Match` holds `std::vector<Car> cars_`. This gives `CarInput` its obvious home and
deletes the `// TODO belongs here?`.

### 2. Integration as free functions over the `Body` variant

Integration isn't car-specific — it's generic rigid-body work, and the codebase
already `std::visit`s the `Body` variant in the solver. Lean into that:

```cpp
namespace engine::physics {
void ApplyGravity(Box&, glm::vec3 g, float dt);   // + Sphere overload, or template
void IntegrateForces(Box&, float dt);             // accumulators -> velocity, then zero
void IntegrateState(Box&, float dt);              // velocity -> position, w -> orientation
void ApplyDamping(Box&, float lin, float ang, float dt);
}
```

Payoff: the duplicated ball-vs-car blocks in `Tick` (§3) collapse into one uniform
loop, and this **automatically fixes the §2 bug** where only `boxes_[0]` integrated
— now every body integrates. Safe precisely because immovable bodies have
`mass_inv == 0` and zero velocity, so the same loop is a no-op for them.

### 3. Collision pair-generation as a free function

Move `GenerateContactConstraints` out as
`collision::GenerateContacts(bodies, walls, ground, params) -> ContactConstraints`.
This is also the seam flagged for the upcoming spatial-partitioning work, so
isolating it now means that swap touches one function, not `Match`.

### How `Tick` reads afterward

```cpp
while (accumulator_ >= fixed_dt) {
  ForEachBody([&](auto& b){ physics::ApplyGravity(b, gravity, fixed_dt); });

  for (Car& car : cars_)
    car.AccumulateDrivingForces(Chassis(car));        // id -> Box&

  ForEachBody([&](auto& b){ physics::IntegrateForces(b, fixed_dt); });

  auto contacts = collision::GenerateContacts(bodies_, walls_, ground_, sim_params_);
  constraint_solver_.PreSolve(bodies_, contacts.normal, contacts.friction, fixed_dt);
  constraint_solver_.Solve(bodies_, contacts.normal, contacts.friction, kSolverIterations);

  ForEachBody([&](auto& b){ physics::IntegrateState(b, fixed_dt); });
  ForEachBody([&](auto& b){ physics::ApplyDamping(b, lin_damp, ang_damp, fixed_dt); });
  accumulator_ -= fixed_dt;
}
```

`ForEachBody` is a one-line `std::visit` loop over `bodies_`. `Tick` becomes a
readable phase list, and each phase is a testable unit.

### What to defer (don't over-build)

The textbook move is a separate `PhysicsWorld` class wrapping the body map, with
`Match` on top owning gameplay/rules. It's a legitimate seam, but **not yet** —
`Match` *is* the world right now, and a wrapper buys nothing until there's real
gameplay (goals, scoring, reset rules) sitting above the physics. Revisit then.

### Order

1. **Integration free functions** — mechanical, no behavior change beyond fixing
   the all-bodies integration; do first to kill the `Tick` duplication.
2. **`Car` entity** — the real decomposition; removes `boxes_[0]`, houses
   `CarInput` + `CarParams`.
3. **Collision pair-gen extraction** — sets up the broad-phase work.

Each step is small and independently testable, and `Match` ends up roughly half
its current size without a single speculative abstraction.
