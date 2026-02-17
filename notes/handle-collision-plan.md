# HandleCollision Plan

## Context
- `ComputeContact` and `ResolveCollision` are separate, and the caller must get the argument order and normal direction right
- The contact normal direction isn't consistent — it depends on which box becomes the reference face during clipping
- This means `ResolveCollision` can silently produce wrong results if the normal points the wrong way
- The Box/Sphere resolve has an `assert(rel_vel_along_normal <= 0)` guard; Box/Box does not

## Goal
Add a `HandleCollision(Box&, Box&)` function that encapsulates the full collision pipeline, enforcing correct argument order and normal direction internally.

## Steps

### 1. Fix the normal convention coming out of ComputeContact
- Decide on a convention: normal points from box_b toward box_a (matching `rel_vel = v_a - v_b`)
- After `ComputeContact` returns, ensure the normal follows this convention relative to the two boxes passed in
- This might mean flipping the normal based on which box became ref vs inc internally

### 2. Add separating velocity assert to Box/Box ResolveCollision
- Add `assert(v_n <= 0.0f)` after line 415 (matching the Box/Sphere version at line 296)
- This will catch normal direction bugs early in debug builds

### 3. Create HandleCollision
- Signature: `void HandleCollision(Box& box_a, Box& box_b)`
- Internally: call `ComputeContact`, ensure normal points from b toward a, call `ResolveCollision`
- Caller doesn't need to know about ref/inc or normal direction

### 4. Add HandleCollision tests
- Two boxes head-on (basic sanity)
- Swapped argument order produces same result
- Immovable + movable, both argument orders
- Off-center impact produces expected rotation direction
- Energy conservation (elastic)

### 5. Clean up
- Mark the separating velocity bug as resolved in `box-box-separating-velocity-bug.md`
- Update `session-goal.md`
- Existing ComputeContact/ResolveCollision tests stay as-is
