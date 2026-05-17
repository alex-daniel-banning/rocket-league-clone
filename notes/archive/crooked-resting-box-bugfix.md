## Uneven Cube Sinking (corners crooked, one side sunk more than others) — FIXED
Root cause: bug in `collisions.cpp` ClipFaceFace `has_parallel` branch. When `axis_source == FACE_B`, the reference/incident bodies were not swapped — box_a was always used as the clipping reference. This meant contact points were on the ground's flat surface (all same height), giving equal penetration depths for all 4 corners. The solver applied equal correction to all points, preserving the tilt.

Fix: added the same FACE_A/FACE_B swap logic that `has_perpendicular` and `ClipCornerToFace` branches already had. Now when the ground's face wins SAT, the tilted box's corners become the incident vertices with varying depths, and the solver generates corrective torque to level the box.
