# Session TODO

## Warm Starting Validation
- [ ] Add convergence logging to the solver: track total impulse delta across all constraints per iteration
- [ ] Compare convergence curve with warm starting on vs. off (disable by setting accumulated_impulse = 0)
- [ ] If deltas are similar, iteration count is already sufficient — warm starting will matter more with many simultaneous contacts

## Uneven Cube Sinking (corners crooked, one side sunk more than others)
Root cause: position correction not fully resolving penetration across all contact points simultaneously. Sequential impulse problem — correcting contact A rotates the body, deepening contact B.

Likely fixes:
- [ ] Increase solver iteration count (more contacts on a body = more iterations needed)
- [ ] Increase bias factor (more aggressive penetration correction per frame)
- [ ] Decrease slop (less allowed penetration before correction kicks in)

## Epsilon Tuning (contact_constraint_solver.cpp:140)
- Current value: 0.01 (placeholder)
- Recommended range: 0.05–0.2 based on simulation scale (ball radius=1.0, dt=1/120, typical velocities 20-40 units/s)
- Too small → warm starting misses matches, solver needs more iterations
- Too large → inherits impulses from wrong contact point, causes jitter
