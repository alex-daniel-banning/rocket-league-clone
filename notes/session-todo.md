# Session TODO

- [ ] **Remove dead `CalculateCentroid`** — `src/engine/physics/collisions.cpp:10`.
      Orphaned by the manifold + per-point resolution refactor. Its only call site
      was the old single-impulse resolver (`collisions.cpp:298` at `4d2304a`,
      `impulse_centroid = CalculateCentroid(contact.points)`); per-point resolution
      removed that, but the function itself was left behind. No remaining callers
      at `master` — verify with `git grep -n CalculateCentroid` before deleting.
