# Split Impulse

## What it does
Separates position correction (penetration resolution) from velocity solving.
Instead of applying Baumgarte stabilization through the velocity solver — which
adds artificial energy — position errors are corrected with a dedicated
"pseudo-velocity" that only affects position integration, not real velocity.

## Why it helps
Baumgarte stabilization works by adding a bias term to the velocity constraint:

```
velocity_bias = (baumgarte / dt) * penetration_depth
```

This pushes objects apart, but the added velocity persists beyond what's needed
for separation. The result: objects on resting surfaces bounce or jitter because
the solver keeps injecting energy to fix tiny penetrations.

Split impulse fixes this by maintaining two separate velocities per body:
- **Real velocity**: only affected by physical impulses (normal + friction)
- **Pseudo-velocity**: only affected by position correction impulses

Position integration uses `real_velocity + pseudo_velocity`, but only
`real_velocity` carries forward to the next frame. The position error gets
fixed without contaminating the velocity state.

## Implementation steps

### 1. Add pseudo-velocities to Box and Sphere
`Body` is a `std::variant<Box*, Sphere*>`, so add `pseudo_linear_velocity` and
`pseudo_angular_velocity` fields to both `Box` and `Sphere`, initialized to
zero each substep.

```cpp
// In both Box and Sphere:
glm::vec3 pseudo_linear_velocity{0.0f};
glm::vec3 pseudo_angular_velocity{0.0f};
```

### 2. Split the bias in GenerateFromContact
Currently the velocity bias combines restitution bounce and Baumgarte
penetration correction into one term. Split it into two:

```cpp
struct ContactConstraint {
  // existing fields...
  float velocity_bias;   // restitution bounce only
  float position_bias;   // Baumgarte penetration correction only
};
```

In `GenerateFromContact`:
```
float restitution_bias = restitution * closing_velocity;  // only if separating
float position_bias = (baumgarte / dt) * max(penetration - slop, 0.0f);
```

The `slop` term (small tolerance, e.g. 0.005) prevents the solver from fighting
over negligible penetrations.

### 3. Modify the solver loop
In `Solve`, when computing the impulse for each constraint:

**Normal impulse (real velocity):**
```
jv = ComputeJV(bodies, cc);            // uses real velocity
lambda = -(jv + cc.velocity_bias) / cc.effective_mass;
// clamp accumulated impulse >= 0
ApplyImpulse(bodies, cc, clamped_lambda);  // updates real velocity
```

**Position correction (pseudo-velocity):**
```
pseudo_jv = ComputePseudoJV(bodies, cc);   // uses pseudo velocity
pseudo_lambda = -(pseudo_jv + cc.position_bias) / cc.effective_mass;
// clamp accumulated pseudo impulse >= 0
ApplyPseudoImpulse(bodies, cc, clamped_pseudo_lambda);  // updates pseudo velocity
```

### 4. Add ApplyPseudoImpulse and ComputePseudoJV
These mirror `ApplyImpulse` and `ComputeJV` but operate on pseudo-velocities
instead of real velocities.

```cpp
void ApplyPseudoImpulse(Bodies& bodies, const ContactConstraint& cc, float lambda) {
  // same math as ApplyImpulse but writes to pseudo_linear/angular_velocity
}
```

### 5. Update position integration
In the integration step, use the combined velocity for position but only carry
real velocity forward:

```cpp
body.position += (body.linear_velocity + body.pseudo_linear_velocity) * dt;
body.orientation += angular_to_quat(body.angular_velocity + body.pseudo_angular_velocity) * dt;

// reset pseudo-velocities for next substep
body.pseudo_linear_velocity = glm::vec3(0.0f);
body.pseudo_angular_velocity = glm::vec3(0.0f);
```

### 6. Remove Baumgarte from velocity bias
Once split impulse is working, the `velocity_bias` in `GenerateFromContact`
should no longer include the Baumgarte term — only the restitution bounce.
Penetration correction is handled entirely through pseudo-velocities.

## Interaction with warm starting
Split impulse and warm starting are complementary. Warm starting seeds the
real-velocity impulse accumulator. The pseudo-velocity accumulator should NOT
be warm started — it resets each substep since position errors change
frame-to-frame.

## Testing
- **SplitImpulse_RestingContact_NoJitter**: Box resting on ground for many
  substeps. Real velocity should converge to near-zero and stay there (no
  oscillation from Baumgarte energy injection).
- **SplitImpulse_BouncingBall_RestitutionPreserved**: Ball dropped onto ground
  with restitution < 1. Should bounce to correct height — split impulse must
  not eat the restitution response.
- **SplitImpulse_PenetrationResolved**: Two overlapping boxes should separate
  over a few substeps without gaining real velocity.
- **SplitImpulse_WithWarmStarting**: Confirm warm starting still converges
  faster when split impulse is active.

## Tuning parameters
- **Baumgarte factor**: Start with 0.1-0.2. Higher values correct penetration
  faster but can cause visible popping if too aggressive.
- **Slop**: 0.001-0.01. Prevents micro-corrections on contacts that are
  essentially resolved. Too large and objects visibly overlap; too small and
  the solver fights over noise.

## Gotchas
- Friction impulses should only use real velocity, not pseudo-velocity.
  Friction opposes real sliding motion, not position correction.
- If using substeps, pseudo-velocities must be reset between substeps, not
  just between frames.
- The pseudo-impulse needs its own accumulated lambda for clamping, separate
  from the real impulse accumulator.
