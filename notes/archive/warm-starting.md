# Warm Starting

## What it does
Seed each frame's constraint solver with last frame's accumulated impulse so it
starts near the converged solution instead of from zero. This dramatically
improves resting contact stability (objects settling on the ground, stacking).

## Why it helps
Without warm starting, 10 solver iterations start from scratch every substep.
For resting contacts the correct impulse is nearly identical frame-to-frame, so
re-discovering it wastes most of those iterations. Warm starting lets the solver
spend its iterations refining rather than catching up.

## Implementation steps

## Testing
- **WarmStarting_ConvergesFaster**: Run solver with and without warm start on
  identical resting scenario. With warm start, fewer iterations should reach
  the same result.
- **WarmStarting_RestingContact_StaysStable**: Ball resting on ground over many
  substeps — velocity should stay near zero, no drift or jitter.

## Gotchas
- When a contact disappears (objects separate), its cached impulse is simply
  dropped — no cleanup needed.
