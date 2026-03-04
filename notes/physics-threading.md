# Physics Threading (Physics Server)

## Current Architecture

Physics and rendering run sequentially on the same thread:

```
[measure dt] → [match.Tick(dt)] → [render] → [swap buffers] → repeat
```

`Match::Tick` already uses a fixed-timestep accumulator internally, so physics
steps at a consistent `fixed_dt` rate. However, because everything is on one
thread, an expensive render frame delays the next physics step, and vice versa.

## What a Physics Thread Would Look Like

Physics runs on a dedicated thread, advancing the simulation at `fixed_dt`
continuously. The render thread samples physics state (with interpolation) and
renders as fast as it wants.

```
Thread A (physics): [step] → [step] → [step] → ...   (fixed_dt, e.g. 1/120s)
Thread B (render):  [sample + interp] → [render] → [swap] → ...   (uncapped)
```

The render thread computes an interpolation factor:

```cpp
float alpha = accumulator / fixed_dt;  // in [0, 1)
rendered_pos = glm::mix(prev_state.pos, curr_state.pos, alpha);
```

This produces smooth visuals even at render rates much higher than the physics
tick rate.

## Advantages

**Consistent physics rate regardless of render cost**
If rendering spikes to 30ms, physics keeps stepping at fixed_dt unaffected.
No more tick death spirals caused by slow render frames.

**Uncapped / higher render framerate**
Render can run at 144Hz+ while physics runs at 60Hz. Currently, render is
bottlenecked by how long physics takes per frame.

**Better CPU utilization**
Physics and rendering run on separate cores in parallel.

**Cleaner separation of concerns**
Physics becomes a pure simulation with no knowledge of rendering. Easier to
test and reason about determinism.

**Foundation for networked play**
A physics server model maps naturally to authoritative server architecture
for multiplayer — physics state is produced independently and clients consume it.

## Challenges

**State synchronization**
The render thread needs a consistent snapshot of physics state to interpolate
from. Options:
- Double-buffer the state (physics writes to back buffer, render reads front)
- Mutex-protected state copy at fixed intervals
- Lock-free ring buffer of state snapshots (most complex, best performance)

**Interpolation adds latency**
Rendering always shows state from one tick in the past. For a game this is
acceptable (~8ms at 120Hz physics), but worth noting.

**Debugging is harder**
Race conditions and non-deterministic ordering make bugs harder to reproduce.
Good logging/tracing becomes more important.

**Input timing**
Input needs to be injected into the physics thread at the right tick boundary,
not at render time.

## Recommended Approach (when ready)

1. Extract physics state into a copyable `PhysicsSnapshot` struct
2. Physics thread produces snapshots into a double buffer
3. Render thread reads the latest complete snapshot + alpha for interpolation
4. Gate behind a feature flag initially to validate correctness against the
   single-threaded baseline
