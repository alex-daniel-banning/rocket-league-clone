# Collision System Refactor

**Motivation:** Jittering/instability with resting positions (boxes on ground).

**Planned upgrades:**
- Constraint solvers / Sequential Impulse (SI)
- Baumgarte stabilization
- Per-point impulse solving/resolution
- Improved contact manifolds (rather than collapsing into single centroid point)
- Warm starting

---

## Branch / PR Strategy

Multiple PRs — these changes are naturally sequential and each can be independently verified. A single PR would be hard to review and hard to bisect if a regression appears.

| PR | Scope |
|----|-------|
| 1 | Improved contact manifolds (detection-side only, no solver changes) |
| 2 | SI + per-point resolution (core solver rewrite — these two are inseparable) |
| 3 | Baumgarte stabilization (targeted change to the constraint equation inside the SI loop) |
| 4 | Warm starting (caching logic on top of the stable solver) |

---

## Execution Order

Contact manifolds are a **detection** concern; the rest are **resolution** concerns. Detection comes first so the solver has good data to work with.

### 1. Improved contact manifolds
The SAT already generates multiple contact points via Sutherland-Hodgman clipping (`ClipFaceFace`, etc.), but the resolution code currently applies a single impulse at a single point. Before rewriting the solver, ensure manifold data is stored and passed correctly for per-point use.

### 2. SI + per-point resolution
Rewrite the resolver to iterate over all contact points individually in a loop (10–20 solver iterations per substep). This is the hardest step and the foundation for everything else. Primary targets: `ResolveBoxBoxCollision` / `ResolveBoxSphereCollision`.

### 3. Baumgarte stabilization
Once SI is working, add the position correction bias term `(β/dt) * C` into the velocity constraint inside the SI loop. This replaces/augments the current positional correction hack. Typical `β` value: ~0.2.

### 4. Warm starting
Cache the accumulated impulse per contact point between frames and pre-apply it at the start of the next frame's SI loop. Pure convergence optimization — correctness shouldn't depend on it.

---

## Testing Strategy

The project already has a physics test suite. Each PR maps to specific test additions.

### PR 1 — Contact manifolds
- Assert that face-face collisions between resting boxes return 4 contact points (not 1–2)
- Assert contact point positions are geometrically correct (on the contact plane, within both shapes)
- Purely geometric assertions, no physics needed

### PR 2 — SI + per-point
- **Resting box test**: Place a box on the ground, simulate ~2 seconds, assert position converges and velocity magnitude drops below a threshold — direct regression test for the jitter problem
- **Stacked boxes**: Two boxes stacked, assert both settle stably
- Update existing impulse/momentum conservation tests to work with the new solver loop

### PR 3 — Baumgarte
- Assert that penetration depth after N frames is smaller with Baumgarte enabled vs. disabled
- Assert the `β` bias parameter doesn't cause instability at its upper bound

### PR 4 — Warm starting
- Convergence test: assert a scenario that takes N iterations to settle without warm starting takes fewer with it
- Regression: ensure all PR 2/3 tests still pass (warm starting shouldn't change final state, only convergence speed)

### Visual smoke test (each PR)
Drop a car onto the ground plane and watch it come to rest — jitter should visibly reduce after PR 2 and be essentially gone after PR 3.

---

## Future consideration: Object sleeping (post-PR 4)

Sleep is a **performance optimization** — it skips integration and collision detection for objects at rest. It does not solve instability directly; if the solver is still generating bad impulses, objects may never meet the sleep threshold or may keep waking themselves up. SI + Baumgarte must address the root cause first.

**How it fits:** Add sleep as a PR 5 after Baumgarte is stable and objects are settling cleanly. At that point, tuning a sleep threshold is straightforward rather than a moving target.

**Warm starting interaction:** When a sleeping object wakes, its cached contact impulses may be stale (contact geometry may have changed). Clear or validate the warm start cache on wake to avoid applying incorrect impulses.

**Contact islands (optional):** A more correct sleep system puts entire groups of touching objects to sleep and wakes them together, preventing cascading wake events. Adds complexity — likely not needed for this project but worth knowing about.
