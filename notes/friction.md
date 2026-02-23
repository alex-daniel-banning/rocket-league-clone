# Friction Implementation Plan

## Background

The current collision resolution only applies a **normal impulse** (`impulse = j * contact.normal`).
For a flat face-on contact, the moment arm `r` is parallel to `n`, so `r × impulse = 0` — no torque,
no spin change. Friction requires a **tangential impulse** at the contact point to oppose sliding
and transfer angular momentum.

---

## Implementation Steps

### 1. Add friction to `ResolveBoxBoxCollision` (`collisions.cpp`)

After the normal impulse block (after the two `ApplyImpulse` calls, before `CorrectPenetration`),
add a tangential impulse pass:

```cpp
// --- Friction ---
glm::vec3 v_tangent = rel_vel - v_n * contact.normal;
float tangent_speed = glm::length(v_tangent);
if (tangent_speed > 1e-6f) {
    glm::vec3 t = v_tangent / tangent_speed;
    float m_eff_t = box_a.mass_inv + box_b.mass_inv
        + AngularMassContribution(i_world_inv_a, r_a, t)
        + AngularMassContribution(i_world_inv_b, r_b, t);
    // Coulomb clamp: friction impulse magnitude <= mu * normal impulse magnitude
    float j_t = glm::clamp(-tangent_speed / m_eff_t, -mu * j, mu * j);
    glm::vec3 friction_impulse = j_t * t;
    ApplyImpulse(box_a.velocity, box_a.mass_inv, box_a.angular_velocity, i_world_inv_a, r_a,  friction_impulse);
    ApplyImpulse(box_b.velocity, box_b.mass_inv, box_b.angular_velocity, i_world_inv_b, r_b, -friction_impulse);
}
```

Note: `rel_vel` must be recomputed *before* the normal impulse is applied (i.e. the pre-impulse
relative velocity), which is already available in the function. The Coulomb clamp uses `j` (the
normal impulse scalar), so keep `j` in scope when you add this block.

### 2. Mirror the change in `ResolveBoxSphereCollision` (`collisions.cpp`)

Same structure. The tangential velocity at the contact point:

```cpp
glm::vec3 v_tangent = v_rel - glm::dot(v_rel, n) * n;
float tangent_speed = glm::length(v_tangent);
if (tangent_speed > 1e-6f) {
    glm::vec3 t = v_tangent / tangent_speed;
    float m_eff_t = sphere.mass_inv + box.mass_inv
        + AngularMassContribution(i_world_inv, r, t);
    float j_t = glm::clamp(-tangent_speed / m_eff_t, -mu * impulse_scalar, mu * impulse_scalar);
    glm::vec3 friction_impulse = j_t * t;
    sphere.velocity += friction_impulse * sphere.mass_inv;
    ApplyImpulse(box.velocity, box.mass_inv, box.angular_velocity, i_world_inv, r, -friction_impulse);
}
```

### 3. Choose where `mu` lives

Two options:

- **Per-object `mu`**: Add a `float friction` field to `Box` and `Sphere`, and combine at contact
  as `mu = sqrt(a.friction * b.friction)` (geometric mean is standard). More flexible, needed for
  the car game eventually (ice patches, rubber ball, etc.).

- **Global constant for now**: A single `static constexpr float kFriction = 0.4f` at the top of
  `collisions.cpp`. Simpler to start, easy to replace later.

  Recommended starting values: `0.3` – `0.5`. Rocket League-ish feel is around `0.3` for boxes.

### 4. Expose `restitution` and `friction` consistently

`HandleCollision` already takes an optional `restitution` param. Consider adding `friction` the
same way so demos/tests can tune it:

```cpp
void Collisions::HandleCollision(Box& a, Box& b, float restitution = 1.0f, float friction = 0.4f);
```

### 5. Add a test

In the box-box collision tests, add a case like:

- Box A: spinning about Y, stationary position, no linear velocity
- Box B: static wall (mass = 0), directly in the path of box A's face
- Advance by a few substeps
- Assert that box A's angular velocity Y component has decreased / reversed in sign
- Assert linear velocity along normal is reversed

---

## Watch-outs

- **`rel_vel` must be the pre-impulse relative velocity** when computing the tangential direction.
  If you recompute it after the normal impulse, the tangent direction can be unstable near zero.

- **Coulomb clamp direction**: `j_t` should oppose sliding, so the sign convention matters.
  `v_tangent` points in the direction of sliding for box_a relative to box_b; the friction
  impulse on box_a should be `-t` direction (opposing its slide). The formula above handles this
  via the negative sign in `-tangent_speed / m_eff_t`.

- **Static vs kinetic friction**: The clamp `|j_t| <= mu * j` implements kinetic friction.
  Full static friction (objects coming to rest against each other) requires an additional
  "sticking" check: if `j_t` without clamping is within the cone, use it directly (static case);
  otherwise clamp (kinetic case). This matters for resting contact once gravity is in.

- **Energy**: Friction should only remove energy. Verify after adding it that total KE trends
  downward or flat in the perf_demo (especially once damping is restored).
