# Game Tick Organization — Review Notes

Review of `src/engine/match.cpp` (`Match::Tick`) on `master`, 2026-07-31.

Context: written while preparing a knowledge-share talk on the box-jitter
refactor. Focus is on how the tick is *organized*, not on hunting bugs.

---

## Verdict

The ordering is correct. It's the canonical sequential-impulse pipeline and it
matches Box2D's island solve almost step for step:

```
ApplyGravity()                        → integrate velocities
GenerateContactConstraints()          → detect + build constraints
PreSolve() / Solve(10)                → warm start, then N-iteration velocity solve
position += dt * EffectiveVelocity()  → integrate positions
damping                               → (see nit #1)
```

The solver sits between velocity integration and position integration, which is
the whole structural point of the refactor. Nothing below is a correctness
problem.

---

## Confirmed: Baumgarte *and* split impulse

Open question from the talk notes was whether the refactor uses split impulse
alone or split impulse + Baumgarte. It's both, and they aren't competing for the
same slot.

From `constraint_solver.cpp`:

```cpp
position_bias = -(baumgarte / dt) * max(penetration - p_slop, 0.0)
```

That's a textbook Baumgarte bias term. But it's applied via `ComputePseudoJV` /
`ApplyPseudoImpulse` onto `pseudo_velocity` / `pseudo_angular_velocity`, and only
rejoins real velocity at integration time through `EffectiveVelocity()`.

**Baumgarte is the bias formula; split impulse is the channel it rides on.**

Phrasing for the talk: *"Baumgarte stabilization on a separate velocity channel
(split impulse), so position correction doesn't inject energy into the real
velocities."*

Caveat: `box.hpp` wasn't read directly (404 on the guessed path), so the exact
composition in `EffectiveVelocity()` — and where `pseudo_velocity` is zeroed each
tick — is inferred from the solver side rather than confirmed.

## Confirmed: the restitution cutoff is the principled `bounce_threshold`

```cpp
restitution_term = (abs(v_n) > 9.8 * dt) ? restitution * v_n : 0.0
```

That threshold is exactly one tick of gravity — the same Δv injected in step 1.
This is the disciplined version of the old `bounce_threshold` hack: the hack
wasn't wrong, it was just unmotivated. The refactor gives it a derivation.

---

## Organizational nits

None of these are correctness bugs. #1 and #2 are the ones worth doing, mostly
because they make the code match the mental model being taught.

### 1. Damping is in the wrong slot

It runs after position integration; it belongs with gravity as part of
"integrate velocities."

Two reasons:

- It makes the loop read as 4 steps instead of 5, matching the canonical
  formulation.
- Applying damping post-solve rescales the velocities the solver just converged
  on, so next tick's warm-started impulses are seeded against velocities that no
  longer match the state they were computed for.

At `0.98`/sec the magnitude is negligible, but the move is free.

### 2. `Tick()` inlines ~25 lines of quaternion integration

Duplicated for `ball_` and `boxes_`. Extract an `IntegratePositions()` to sit
alongside `ApplyGravity()`.

Then `Tick()` becomes five named calls and *is* the presentation slide, verbatim
— no pseudocode translation needed.

### 3. Ball/box duplication throughout

`ball_` and `boxes_` are handled separately in gravity, integration, damping, and
five hand-written collision blocks. A `bodies_` collection already exists (it's
passed to the solver) — routing everything through it would collapse
`GenerateContactConstraints`'s ~65 lines into a single nested loop.

### 4. Tuning constants are function-local literals

Scattered across two files: `baumgarte 0.02`, `restitution 0.3`, `p_slop 0.002`,
`damping 0.98`, `solver_iterations 10`.

Worth a config struct. For the presentation specifically, these five numbers are
exactly what an audience will ask about, so having them in one place is itself a
slide.

### 5. The death-spiral check logs but doesn't clamp

`substeps > 10` warns after the fact; a genuine spiral still hangs the frame. The
conventional fix is capping substeps and discarding the remainder. Off-topic for
the talk, but a real latent issue.

---

## Related: pseudocode framing for the talk

Separate from the code itself — on whether "apply gravity" should be its own step
in the explanatory pseudocode.

Gravity *is* integration. Semi-implicit Euler is two lines:

```
v += (F/m) · dt     ← gravity lives here
x += v · dt
```

So fold gravity into **"integrate velocities"** — but do *not* collapse both
lines into one monolithic "integrate" step, because the entire refactor happens
in the gap between them.

**Before (old engine):**

```
each tick:
  1. v += g·dt                          // gravity
  2. x += v·dt                          // integrate positions
  3. for each colliding pair:           // single pass, pair at a time
       a. normal impulse
       b. friction
       c. teleport apart to remove penetration
  4. v *= damping
```

**After (current):**

```
each tick:
  1. Integrate velocities    v += g·dt;  v *= damping
  2. Detect collisions       build contact manifolds
  3. Solve velocities        N iterations of sequential impulse over ALL
                             contacts — per-point normal + friction,
                             warm-started, Baumgarte bias on the pseudo channel
  4. Integrate positions     x += (v + v_pseudo)·dt
```

Four steps each, same shape, every difference load-bearing:

- Step 3c vanishes entirely. Positional teleports were a patch for solving *too
  late* — after bodies had already moved into each other. Solve before position
  integration and the correction rides the velocity channel instead.
- "For each pair" becomes "N passes over all contacts."
- The solver crosses to the other side of position integration.

Keep `v += g·dt` visible as a literal line rather than hiding it behind
"integrate." A resting box gets a fresh downward Δv injected every single tick,
and the solver's job is to cancel exactly that, every tick, forever. That fact
sets up both the object-sleeping argument and the jitter mechanism itself.
