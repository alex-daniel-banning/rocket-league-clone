- [x] Fix ClipCornerToFace bug (resolved: needed to invert penetration axis when passing box_b, box_a)
- [ ] Audit other clipping functions for the same bug
    ClipFaceFace: line 370 swaps to (box_b, box_a) but doesn't negate penetration_axis — same bug?
    ClipEdgeEdge: line 355 always passes (box_a, box_b) — does it need a swapped path?
    Verify each function's assumption about penetration_axis direction relative to ref/inc params
- [ ] Add HandleCollision function (see handle-collision-plan.md)
    Encapsulates ComputeContact + ResolveCollision with correct normal direction enforcement

