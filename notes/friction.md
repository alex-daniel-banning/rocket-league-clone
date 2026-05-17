# Friction Implementation Plan

## Background

The solver currently handles normal (non-penetration) impulses only. Objects slide
freely along surfaces. Friction adds tangential impulses that resist sliding,
enabling cars to grip the ground and the ball to roll instead of glide.

An earlier friction implementation was removed when the sequential impulse system
was introduced (commit 38b0c84). This plan re-implements friction within the
constraint-based framework.

### Key constraint from split-impulse notes
> Friction impulses should only use real velocity, not pseudo-velocity.
> Friction opposes real sliding motion, not position correction.

---

## Approach: Coulomb Friction as Tangent Constraints

For each contact point, the solver currently generates **one** `ContactConstraint`
(normal direction). Friction adds **two more** constraints per contact point — one
for each tangent direction in the contact plane. These tangent constraints:

- Share the same contact point, body pair, and inertia data as the normal constraint
- Have Jacobians built from tangent vectors instead of the normal
- Are clamped by the Coulomb friction cone: `|λ_t| ≤ μ * λ_n`
- Participate in the same sequential impulse loop
- Use real velocity only (no pseudo-velocity / no position bias)

This is the standard approach used by Box2D, Bullet, and most iterative solvers.

---

## Implementation Steps

### Step 1 — Introduce a `FrictionConstraint` struct

New file: `include/engine/physics/friction_constraint.hpp`

```cpp
struct FrictionConstraint {
  long body_a_id;
  long body_b_id;
  std::array<float, 12> jacobian;     // tangent direction Jacobian
  float effective_mass;
  glm::mat3 i_world_inv_a;
  glm::mat3 i_world_inv_b;
  float accumulated_impulse = 0.0f;
  float mu;                           // Coulomb friction coefficient for this pair
  glm::vec3 position;                 // for warm starting matching
  int normal_constraint_index;        // index into normal constraints vector
};
```

**Why a separate struct instead of reusing `ContactConstraint`?**
- No `velocity_bias`, `position_bias`, or `pseudo_accumulated_impulse` (friction
  has none of these — it only operates on real velocity)
- Needs a link to its parent normal constraint for Coulomb clamping
- Keeps the normal solve loop unchanged; friction is a clean addition

**Alternative considered**: a single `ContactConstraint` with an `is_friction` flag
and optional fields. Rejected because it complicates the normal solve loop and
wastes memory on unused bias fields.

### Step 2 — Tangent basis computation

In `GenerateFromContact`, after building the normal constraint, compute two
tangent vectors orthogonal to the contact normal:

```
t1 = normalize(v_rel - (v_rel · n) * n)    // aligned with sliding direction
if |t1| ≈ 0:
    t1 = arbitrary_perpendicular(n)         // no sliding → pick any tangent
t2 = cross(n, t1)
```

Always orient `t1` from the sliding velocity. This improves solver convergence
(the primary friction force aligns with sliding, so fewer iterations to cancel
it) and keeps the tangent basis stable across frames for warm starting. For
resting contacts where sliding velocity is near zero, the fallback to an
arbitrary perpendicular may cause the basis to flip between frames, misapplying
warm-started friction impulses. Revisit if jitter appears on resting contacts —
the fix would be to store the previous tangent basis and reuse it when the
contact persists.

### Step 3 — Generate friction constraints

For each contact point, after pushing the normal constraint, push two
`FrictionConstraint`s (one per tangent). The Jacobian is built identically to
the normal constraint but substituting `t` for `n`:

```
J_friction = [-t, -(r_a × t), t, (r_b × t)]
```

Effective mass is computed the same way:
```
w = inv_m_a + inv_m_b + dot(cross(I_a_inv * (r_a × t), r_a), t)
                      + dot(cross(I_b_inv * (r_b × t), r_b), t)
effective_mass = 1 / w
```

Store the index of the corresponding normal constraint so the solve loop can
read `λ_n` for Coulomb clamping.

### Step 4 — Add `friction` field to `Box` and `Sphere`, update `GenerateFromContact`

Add a `const float friction` field to both `Box` and `Sphere`:

```cpp
// box.hpp
const float friction = 0.5f;

// sphere.hpp
const float friction = 0.5f;
```

These are const (like `mass`) since friction is an intrinsic material property.
Default 0.5 is a reasonable middle ground. Constructors and `BoxBuilder` need
updating to accept the new field.

`GenerateFromContact` computes the pair friction from the two bodies instead of
receiving it as a parameter:

```cpp
static void GenerateFromContact(
    const Contact& contact,
    const std::unordered_map<int, Body>& bodies,
    float dt,
    std::vector<ContactConstraint>& out,
    std::vector<FrictionConstraint>& friction_out,  // new
    float restitution = 1.0f,
    float baumgarte = 0.2f);
```

Inside the function, read friction from both bodies and combine with the
geometric mean:

```cpp
float mu_a = std::visit([](auto* body) { return body->friction; }, bodies.at(contact.body_a_id));
float mu_b = std::visit([](auto* body) { return body->friction; }, bodies.at(contact.body_b_id));
float mu = std::sqrt(mu_a * mu_b);
```

When `mu > 0`, generate friction constraints. When `mu == 0` (either body is
frictionless), skip them entirely (no overhead for frictionless contacts).

### Step 5 — Solve friction in the iterative loop

In `Solve()`, after the normal impulse for a contact point, solve its two
friction constraints:

```
for each friction constraint fc:
    jv = ComputeJV(bodies, fc)          // real velocity only, no bias
    lambda = -jv * fc.effective_mass
    // Coulomb clamp
    max_friction = mu * normal_constraints[fc.normal_constraint_index].accumulated_impulse
    old_accum = fc.accumulated_impulse
    fc.accumulated_impulse = clamp(old_accum + lambda, -max_friction, max_friction)
    delta = fc.accumulated_impulse - old_accum
    ApplyImpulse(bodies, fc, delta)
```

Key details:
- **Clamp is symmetric** (`-max_friction` to `+max_friction`) because friction
  opposes motion in either tangent direction
- **Uses `accumulated_impulse` from the normal constraint** (not raw lambda),
  so the friction bound tightens/loosens as the normal force evolves across
  iterations
- **No position bias or pseudo-velocity** — friction only touches real velocities
- Order matters: solve normal first, then friction, so the Coulomb bound is
  up-to-date

### Step 6 — Warm starting for friction constraints

Extend `PreSolve` to also warm-start friction constraints. The matching logic is
the same: match by body pair + closest contact position. Reuse the previous
frame's `accumulated_impulse` for each friction constraint.

This is critical for resting contacts — without warm starting, friction has to
re-derive the full static friction impulse every frame, causing visible jitter.

Store `previous_friction_constraints_` alongside `previous_constraints_` in the
solver.

### Step 7 — Set friction coefficients on bodies in `Match`

Since friction is now a per-body property, set it when constructing each object:

| Body             | Suggested μ |
|------------------|-------------|
| Ground           | 0.8         |
| Walls            | 0.3         |
| Cars             | 0.6         |
| Ball             | 0.5         |

Resulting pair values (geometric mean `sqrt(μ_a * μ_b)`):

| Contact pair     | Effective μ |
|------------------|-------------|
| Car v Ground     | 0.69        |
| Ball v Ground    | 0.63        |
| Ball v Wall      | 0.39        |
| Car v Wall       | 0.42        |
| Car v Car        | 0.60        |
| Ball v Car       | 0.55        |

These are starting points to tune visually. Rocket League uses very
game-specific friction curves, but constant Coulomb friction is the right
foundation.

### Step 8 — Tests

Port the old friction test ideas to the constraint-based system:

1. **Sliding on a slope**: box on a tilted surface with μ chosen so it should
   remain stationary (μ > tan(θ)). Verify velocity stays near zero.
2. **Sliding deceleration**: sphere moving horizontally on a flat surface.
   Verify it decelerates and eventually stops.
3. **Friction vs. frictionless**: same scenario with μ=0 and μ=0.5. Verify μ=0
   slides indefinitely; μ=0.5 decelerates.
4. **Rolling**: sphere on a surface with friction should develop angular
   velocity (spin). Without friction, no spin.
5. **Coulomb bound**: verify that friction impulse magnitude never exceeds μ *
   normal impulse.

---

## Files to modify

| File | Change |
|------|--------|
| `include/engine/physics/friction_constraint.hpp` | **New** — struct definition |
| `include/engine/physics/contact_constraint_solver.hpp` | Add friction vectors, update signatures |
| `src/engine/physics/contact_constraint_solver.cpp` | Generate, warm-start, and solve friction |
| `include/engine/physics/box.hpp` | Add `const float friction` field, update constructor |
| `include/engine/physics/sphere.hpp` | Add `const float friction` field, update constructor |
| `src/engine/match.cpp` | Pass friction values when constructing bodies, add `friction_out` vector |
| `tests/src/collision_tests/` | New friction test file |

---

## Shared impulse application via templates

`FrictionConstraint` and `ContactConstraint` share the same fields used by
`ApplyImpulse` and `UnpackJacobian`: `body_a_id`, `body_b_id`, `jacobian`,
`i_world_inv_a`, `i_world_inv_b`. Template these functions so both constraint
types work without duplication or extra parameters.

`UnpackJacobian` currently takes `const ContactConstraint&` — change it to:

```cpp
template <typename Constraint>
JacobianComponents UnpackJacobian(const Constraint& c) {
  return {
      glm::vec3(c.jacobian[0], c.jacobian[1], c.jacobian[2]),
      glm::vec3(c.jacobian[3], c.jacobian[4], c.jacobian[5]),
      glm::vec3(c.jacobian[6], c.jacobian[7], c.jacobian[8]),
      glm::vec3(c.jacobian[9], c.jacobian[10], c.jacobian[11]),
  };
}
```

`ApplyImpulse` (and `ApplyPseudoImpulse` if needed) become:

```cpp
template <typename Constraint>
void ApplyImpulse(std::unordered_map<int, Body>& bodies,
                  const Constraint& c, float lambda) {
  const auto j = UnpackJacobian(c);
  std::visit([&](auto* body) {
    body->velocity += body->mass_inv * j.j_v_a * lambda;
    body->angular_velocity += c.i_world_inv_a * j.j_w_a * lambda;
  }, bodies[c.body_a_id]);
  std::visit([&](auto* body) {
    body->velocity += body->mass_inv * j.j_v_b * lambda;
    body->angular_velocity += c.i_world_inv_b * j.j_w_b * lambda;
  }, bodies[c.body_b_id]);
}
```

Both stay in the anonymous namespace of `contact_constraint_solver.cpp`. The
existing call sites (`ApplyImpulse(bodies, cc, delta)`) don't change — the
compiler deduces the template parameter. `ComputeJV` should also be templated
since friction uses the same Jacobian dot-product computation against real
velocities.

`ApplyPseudoImpulse` does **not** need to be templated — friction constraints
never use pseudo-velocities.

These functions also need to move from private static methods on
`ContactConstraintSolver` to free functions in the anonymous namespace (they're
already in the .cpp file, so no visibility change). This is required because
class member functions can't be templates without the class itself being
templated or the template being declared in the header. Moving them to the
anonymous namespace is cleaner and they were already implementation details.

---

## Open questions

None currently.
