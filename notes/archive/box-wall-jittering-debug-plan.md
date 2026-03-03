# Box-Wall Jittering Debug Plan

## V2 — Collision Stepping

### Core idea

The callback blocks inside `match.Tick()` → `HandleCollisions()`, spinning on `glfwPollEvents()` until the user presses N. The render loop is frozen during this time, but the last rendered frame stays on screen (the compositor holds it) and input events still fire because you're pumping `glfwPollEvents()` inside the spin loop.

### Step 1 — Debug state in `perf_demo.cpp`

Replace the current `DebugMarker` callback with this:

```cpp
static struct {
  bool step_mode = false;
  bool advance   = false;
} g_debug;

static void KeyCallback(GLFWwindow*, int key, int /*scancode*/, int action, int /*mods*/) {
  if (key == GLFW_KEY_L && action == GLFW_PRESS) {
    g_debug.step_mode = !g_debug.step_mode;
    LOG_INFO(g_debug.step_mode ? "=== STEP MODE ON ===" : "=== STEP MODE OFF ===");
  }
  if (key == GLFW_KEY_N && action == GLFW_PRESS) {
    g_debug.advance = true;
  }
}
```

### Step 2 — Add a collision callback to `Match`

In `match.hpp`, add a `#include <functional>` and a setter + member:

```cpp
#include <functional>

// in the public section:
void SetCollisionCallback(std::function<void(const std::string&, const std::string&)> cb) {
  collision_callback_ = std::move(cb);
}

// in the private section:
std::function<void(const std::string&, const std::string&)> collision_callback_;
```

### Step 3 — Fire the callback in `HandleCollisions`

In `match.cpp`, call `collision_callback_` before each `HandleCollision` call. Only needed for
box-vs-wall and box-vs-ground since that's where the bug is:

```cpp
// Car v Wall collisions
for (physics::Box& car : boxes_) {
  for (physics::Box& wall : walls_) {
    if (collision_callback_) collision_callback_(car.name, wall.name);
    physics::Collisions::HandleCollision(car, wall);
  }
}
// Car v Ground collisions
for (physics::Box& car : boxes_) {
  if (collision_callback_) collision_callback_(car.name, ground_->name);
  physics::Collisions::HandleCollision(car, *ground_);
}
```

### Step 4 — Wire it up in `perf_demo.cpp`

After building the match, register the blocking callback:

```cpp
match.SetCollisionCallback([&](const std::string& a, const std::string& b) {
  if (!g_debug.step_mode) return;
  LOG_INFO("--- next: %s vs %s (N to advance) ---", a.c_str(), b.c_str());
  while (!g_debug.advance) {
    glfwPollEvents();
  }
  g_debug.advance = false;
});
```

### Step 5 — Prevent delta_time ballooning after unblocking

When unblocking after a long pause, the next `delta_time` will be enormous and cause a substep
death spiral. Cap it:

```cpp
// top of the render loop, after computing delta_time:
delta_time = std::min(delta_time, 0.1f);  // cap at 100ms = ~12 substeps
```

### Caveat

The callback fires on every pair, not just pairs with actual contact. With one box and 7 surfaces
(6 walls + ground) that's 7 calls per substep — manageable. If it's too noisy, filter by calling
`ComputeContact` explicitly in `HandleCollisions` and skipping pairs that return false.

---

## V1 — Logging + Debug Marker (superseded)

### Plan feedback

**Step 1 & 2 (color + string ID):** Color is a render concern and doesn't belong in `Box`
philosophically — but pragmatically, a `std::string name` field on `Box` (used as the logging ID)
is fine, and the demo can just map that name to a render color at draw time.

**Step 3 (log box IDs in collision resolutions):** Works cleanly since the resolve functions
receive `Box&` directly — if the name lives on `Box`, it's automatically there. The existing
`LOG_DEBUG` calls in `collisions.cpp` just need the names threaded in.

**Step 4 (marker key):** `ProcessInput` polls `glfwGetKey` every frame, so a held key fires every
frame. Use a GLFW key callback (`glfwSetKeyCallback`) instead; it fires once per press with edge
detection built in.

### What to log per collision resolution

| Field | Why it matters |
|---|---|
| Box names (`box_a.name`, `box_b.name`) | Identify which pair is resolving |
| Contact normal | Hypothesis C — detect flickering normals between substeps |
| Penetration depth | Hypothesis A — oscillating corrections leave a trace |
| Contact point count | Hypothesis D — if it flips between 2 and 4, angular impulse is unstable |
| `v_n` (relative velocity along normal) | Verify the "already separating" guard is firing correctly |
| Impulse `j` | A huge or negative impulse is a red flag |
| Box position after correction | Hypothesis A — position bouncing back and forth across substeps |

### What was implemented

1. `std::string name` added to `Box`
2. `BoxBuilder` updated with `.Name(...)` method
3. All boxes named in the demo; names mapped to colors at render time
4. Box names threaded into `LOG_DEBUG` lines in `collisions.cpp`
5. `glfwSetKeyCallback` emitting `LOG_INFO("=== MARKER ===")` on `L`
