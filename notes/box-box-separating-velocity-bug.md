# Box/Box Separating Velocity Bug

## Summary

`ResolveCollision` (Box/Box) applies an attractive impulse when the contact
point is already separating. This causes a rotating box to get repeatedly
pulled into the ground instead of bouncing away.

## How to reproduce

```
./bin/box-box-collision-demo rotation_only
```

Watch the debug logs — the box repeatedly collides with the ground every frame,
with y-velocity oscillating but never flipping positive.

## Root cause

The impulse formula in `ResolveCollision` computes relative velocity at the
**contact point** (including angular velocity contribution), not just the
center-of-mass velocity. When a box is rotating, a corner's velocity can be
moving away from the surface (separating) even though the center of mass is
still approaching. In this case `v_n` flips sign and the impulse formula
produces an impulse that pulls the objects together.

The Box/Sphere `ResolveCollision` already guards against this:

```cpp
// collisions.cpp:289
assert(rel_vel_along_normal <= 0.0f);
```

The Box/Box version has no such guard.

## Fix

Add an early return in Box/Box `ResolveCollision` when contact-point relative
velocity along the normal indicates separation:

```cpp
float v_n = glm::dot(rel_vel, contact.normal);
if (v_n > 0.0f) return;  // already separating at contact point
```

(Exact sign depends on normal convention — normal currently points A→B.)

## Notes

- The normal convention (A→B on line 335) is **correct** — flipping it breaks
  existing tests. The issue is the missing separating-velocity guard, not the
  normal direction.
- Existing tests don't catch this because they use equal-mass boxes where the
  bug is symmetric and invisible. A test with an immovable floor + rotating box
  would catch it.
- Also remove the `static int count = 0; if (count++ > 5) exit(0);` debug code
  in `ResolveCollision` (line 381-382).
