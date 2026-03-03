# Box-Wall Jittering Bug

Boxes in perf_demo twitch and lose energy unnaturally on wall contacts. Confirmed pre-existing
(not caused by friction). Single box demo is fine — issue emerges with multiple boxes.

## Isolation Steps

### Step 1 — Two boxes, no ball
Does twitching occur when hitting walls, or only when the two boxes hit each other?

- Twitching on wall hit → multi-contact position correction conflict (see hypothesis A)
- Twitching only on box-box hit → box-box resolution corrupting state before wall hit (see hypothesis B)

### Step 2 — One box, two walls close together
Does a single box jitter when caught between two walls?

- Yes → confirms hypothesis A (conflicting position corrections, no box-box needed to reproduce)
- No → box-box interaction is required to trigger it

## Hypotheses

### A — Multi-contact position correction conflicts
Two wall collisions both call `CorrectPenetration` on the same box in one substep. The corrections
are applied sequentially and are not aware of each other, so they can push the box in conflicting
directions and cause oscillation.

### B — Box-box resolution corrupting velocity before wall hit
Sequential impulse resolution means resolving pair (A, B) changes A's velocity. When A then hits
a wall later in the same substep, the wall collision sees an already-modified velocity that may
appear to be separating, causing the impulse to be skipped or mis-sized.

### C — Unimplemented edge-edge bias (normal flickering)
`ComputeContact` has a comment saying edge-edge overlaps should be biased toward face axes, but
the bias is not implemented. With multiple boxes at various angles, edge-edge contacts are common
and the contact normal can flicker between substeps. The fix is to discount edge-edge overlaps
slightly before comparing:

```cpp
float biased_overlap = (i >= 6) ? overlap * 0.99f : overlap;
if (biased_overlap < penetration) {
    penetration = biased_overlap;
    // use actual (unbiased) overlap for penetration depth in resolution
}
```

### D — Varying contact point count between substeps
`ClipFaceFace` can return 2 points one frame and 4 the next for a near-parallel contact. The
centroid jumps between substeps, causing `r` to jump, causing angular impulses to vary wildly.
