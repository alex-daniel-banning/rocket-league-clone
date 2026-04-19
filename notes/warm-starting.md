# Warm Starting

## What it does
Seed each frame's constraint solver with last frame's accumulated impulse so it
starts near the converged solution instead of from zero. This dramatically
improves resting contact stability (objects settling on the ground, stacking).

## Why it helps
Without warm starting, 10 solver iterations start from scratch every substep.
For resting contacts the correct impulse is nearly identical frame-to-frame, so
re-discovering it wastes most of those iterations. Warm starting lets the solver
spend its iterations refining rather than catching up.

## Implementation steps

### 1. Persist constraints across frames
Currently `GenerateContactConstraints` creates a fresh `std::vector<ContactConstraint>`
every substep. Instead, store `previous_constraints_` as a member on `Match`.

### 2. Build a lookup key for matching constraints
A contact constraint is identified by the pair `(body_a_id, body_b_id)` plus
which contact point it corresponds to. Define a key type:
```
struct ConstraintKey { int body_a, body_b; int point_index; };
```
After generating this frame's constraints, build a map from key → previous
`accumulated_impulse`. The simplest initial approach: match on `(min(a,b), max(a,b), point_index)`.

### 3. Implement `PreSolve` — apply cached impulse
In `PreSolve`, for each new constraint:
- Look up the matching previous constraint
- If found, set `cc.accumulated_impulse = prev.accumulated_impulse`
- Apply that impulse immediately via `ApplyImpulse`
- If not found, leave `accumulated_impulse = 0` (cold start)

### 4. Save constraints after solve
After `Solve` returns, copy the current constraints into `previous_constraints_`
so next frame can use them.

### 5. Update the game loop (`Match::Tick`)
```
constraints = GenerateContactConstraints(dt);
PreSolve(bodies_, constraints, previous_constraints_);  // warm start
Solve(bodies_, constraints, iterations);
previous_constraints_ = constraints;                    // save for next frame
// integrate positions...
```

## Testing
- **WarmStarting_ConvergesFaster**: Run solver with and without warm start on
  identical resting scenario. With warm start, fewer iterations should reach
  the same result.
- **WarmStarting_RestingContact_StaysStable**: Ball resting on ground over many
  substeps — velocity should stay near zero, no drift or jitter.

## Gotchas
- Contact point ordering may differ frame-to-frame for face-face manifolds.
  `ReduceManifold` should produce stable ordering, but verify. If points shuffle,
  the warm-started impulse lands on the wrong point and can cause a pop.
- When a contact disappears (objects separate), its cached impulse is simply
  dropped — no cleanup needed.
- `GenerateFromContact` currently takes `bodies` by value. Consider changing to
  `const&` to avoid copying the map every call (unrelated but good cleanup).
