# Session TODO

## Epsilon Tuning (contact_constraint_solver.cpp:140)
- Current value: 0.01 (placeholder)
- Recommended range: 0.05–0.2 based on simulation scale (ball radius=1.0, dt=1/120, typical velocities 20-40 units/s)
- Too small → warm starting misses matches, solver needs more iterations
- Too large → inherits impulses from wrong contact point, causes jitter
