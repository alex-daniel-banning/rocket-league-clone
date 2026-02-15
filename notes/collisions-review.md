# Collisions Code Review

## 1. Refactoring

**`CalculateCentroid` (lines 9-22)** — The component-by-component accumulation can be simplified:
```cpp
glm::vec3 sum(0.0f);
for (const auto& p : points) sum += p;
return sum / static_cast<float>(points.size());
```

**`GetAxesFromQuaternion` (lines 24-46)** — Manually extracting rotation matrix columns. `glm::toMat3(q)` does exactly this, then just take `mat[0]`, `mat[1]`, `mat[2]`. The manual math is error-prone and harder to verify.

**`ClipFaceFace` incident axis search (lines 92-116)** — The best axis is found on lines 92-102, then the work is redone on lines 104-116 to find the top two. `second_inc` is never used anywhere. This whole block (104-116) appears to be dead code — remove it.

**Duplicated resolve logic** — The Box/Sphere and Box/Box `ResolveCollision` functions share the same impulse + penetration correction structure. Not urgent, but if friction is added later, avoid duplicating that in both places.

## 2. Clarity

**`ClipFaceFace` comment on line 150-152** — Copy-pasted from line 134-136, says "inc_axis" but refers to `ref_sign`/`ref_dot`. Misleading.

**`ComputeContact` box-box (lines 386-404)** — The `has_parallel`/`has_perpendicular` branching logic is hard to follow. It's not obvious why `has_parallel` always calls `ClipFaceFace(box_a, box_b, ...)` without swapping based on `axis_source`, while `has_perpendicular` does swap. A brief comment explaining the geometric reasoning would help.

**Lines 336, 467** — `return;` at end of void functions is unnecessary.

## 3. C++ Specifics

**`Contact` parameter passing** — Inconsistent across the header. Box/Sphere `ResolveCollision` takes `Contact contact` (by value), Box/Box takes `const Contact contact`. Neither is modified, so both should be `const Contact&` to avoid copying the vector of contact points inside it.

**`ResolveElasticCollision` signatures** — Same inconsistency: Box/Sphere version takes `Contact contact`, Box/Box takes `const Contact contact`. Should both be `const Contact&`.

**`ClipPolygonAgainstPlane` (line 71-72)** — `v1` and `v2` are copies of `glm::vec3`. Fine for vec3 (trivially copyable), but `const auto&` would be more consistent with the style elsewhere (line 169).

**`ComputeContact` box-sphere (line 271)** — `box.Size().x / 2` could use `box.HalfExtents()` like the rest of the codebase does, for consistency.

**`axes_to_test` (line 354)** — `std::vector` with 15 elements allocated on the heap every call. Since it's always at most 15 entries, `std::array` or a small fixed-size container would avoid the allocation.
