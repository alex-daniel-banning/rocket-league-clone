# Raycasting for Wheel/Ground Detection

Notes for the throttle-gating backlog item (4 wheel rays → "are we touching the
ground?" → allow throttle). This is also the foundation for Phase 4 raycasted
suspension in `car-control-and-suspension.md` — build the raycast once, reuse it
for both.

## Is a raycast the same as collision detection?

Same *family* (geometric overlap), but a deliberately different, lighter query.
Per the suspension notes: a wheel is "a spring + damper, **not a collision**."
Do **not** route this through `ComputeContact`.

| | `ComputeContact` (box-box) | Raycast |
|---|---|---|
| Question | Do two volumes interpenetrate? | What's the first surface along a ray, and how far? |
| Output | Full manifold: normal + points + penetration | hit/miss + distance `t` (+ optional point/normal) |
| Cost | Heavy (SAT, clipping) | Cheap (slab test) |
| Purpose | Feed the solver to push bodies apart | A *measurement* — nothing gets resolved |

## How "overlap with a ray" works

A ray is `P(t) = origin + t * dir`, with `t >= 0` (capped at `t_max`). Detecting
overlap means solving for the smallest `t` where `P(t)` lands on the surface.

### Ray vs Box (arena/ground are oriented boxes)

Use the **slab method**. A box is the intersection of three pairs of parallel
planes ("slabs"), one per axis. For each axis compute the `t` range where the ray
is inside that slab; the ray hits the box iff all three ranges overlap.

Because the boxes are *oriented* (they carry a `rotation` quat), transform the ray
into the box's local space first — then it's an AABB and the slab test is trivial.

```cpp
// Ray in world space: origin O, unit dir D, max distance t_max.
// Returns true and sets t_hit if the ray hits the (oriented) box.
bool RaycastBox(const glm::vec3& O, const glm::vec3& D, float t_max,
                const Box& box, float& t_hit) {
  // 1. World -> box-local: undo translation then rotation.
  glm::quat inv = glm::conjugate(box.rotation);
  glm::vec3 lo = inv * (O - box.position);   // local ray origin
  glm::vec3 ld = inv * D;                     // local ray dir (still unit)

  // 2. Slab test against the AABB [-half, +half] in local space.
  glm::vec3 h = box.HalfExtents();
  float tmin = 0.0f, tmax = t_max;
  for (int a = 0; a < 3; ++a) {
    if (std::abs(ld[a]) < 1e-8f) {
      // Ray parallel to slab: miss if origin outside the slab.
      if (lo[a] < -h[a] || lo[a] > h[a]) return false;
    } else {
      float inv_d = 1.0f / ld[a];
      float t1 = (-h[a] - lo[a]) * inv_d;
      float t2 = ( h[a] - lo[a]) * inv_d;
      if (t1 > t2) std::swap(t1, t2);
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);
      if (tmin > tmax) return false;          // slabs don't overlap
    }
  }
  t_hit = tmin;
  return true;
}
```

`tmin` is the entry distance. To check the whole arena, loop over all world boxes
and keep the smallest `t_hit`.

### Ray vs Sphere (if you ever raycast the ball)

Substitute `P(t)` into `|P - center|^2 = r^2` → a quadratic in `t`. Real roots =
hit; smallest non-negative root is the entry point.

## The throttle gate

```cpp
bool wheel_grounded(const glm::vec3& mount, const glm::vec3& down, float reach,
                    const std::vector<Box>& world) {
  float best = reach, t;
  for (const Box& b : world)
    if (RaycastBox(mount, down, reach, b, t)) best = std::min(best, t);
  return best < reach;   // hit something within wheel reach
}
```

Throttle applies if any (or some threshold of) the 4 wheels report grounded.

### Gotchas / decisions

- **Exclude the car's own box** from the world list, or each wheel ray hits the
  car's own hitbox.
- **Return `t` even for the boolean gate.** Phase 4 suspension reuses the *same*
  ray and needs `compression = rest_length - t`. Build it once; the throttle
  check and suspension share it. Returning the surface normal too is cheap and
  useful later (wall/ceiling driving).
- **Cast from a fixed mount point *above* the wheel travel**, not from the contact
  point — keeps the ray origin above the surface under compression so it never
  loses the ground right when the force is needed.

## Testing

Ray-vs-box is pure math with clear answers — good GoogleTest candidate (follow
`tests/src/engine/physics/`): hits vs misses on an axis-aligned *and* a rotated
box, correct distance, ray starting inside, parallel-miss.
