# Raycasted Suspension

## Concept

The car is a single rigid body (one box). The four "wheels" are not physics
objects — they are raycasts that probe the ground each tick and feed spring
forces back into the car body.

```
      car body (one rigid box)
     ┌─────────────────────┐
     │  FL        FR       │
     │   ↓         ↓       │  ← ray origins (wheel mount points)
     └─────────────────────┘
          ↓         ↓
     ─────────────────────────  ground
```

Each tick, each wheel ray is tested against the ground. If the ray hits within
the suspension's rest length, a spring+damper force is computed and applied to
the car body at that mount point.

## Per-Wheel Calculation

```
compression = rest_length - hit_distance
spring_force = k * compression
damper_force = -c * (velocity of car body at mount point along ray direction)
total_force  = spring_force + damper_force   (clamped to >= 0)
```

The force is applied upward at the mount point — `Match::ApplyForceAtPoint` or
equivalent. Because the mount point is offset from the car's center of mass,
this creates both a linear force and a torque, giving natural rocking/pitching
behavior.

## Why This Approach

- **Stable resting & natural feel**: support comes from spring forces, and four
  corner contact points passively resist tipping.
- **No constraint system needed**: tires are not separate physics bodies, so
  there's no need to bind them to the car with rigid body constraints.
- **Natural car feel**: each wheel responds independently to terrain, so the
  car pitches and rolls realistically over uneven surfaces.
- **Standard for arcade car physics**: this is the approach used by most
  arcade/semi-realistic car games including Rocket League.

## Relationship to Current Collision System

Suspension and the box collider coexist — suspension does **not** replace
car-ground collision. In normal driving the springs do all the work: tune the
ride height (`rest_length` vs `k`) so the hitbox floats just above the ground at
equilibrium, leaving no penetration, so the contact solver generates no
car-ground contacts on its own.

Ground collision stays enabled as a backstop for the cases the springs can't
cover:
- **Bottoming out** on a hard landing, when compression maxes out.
- **Flipped / on-side** — the downward wheel rays miss the ground, so without
  collision the car would fall through the floor.

The box collider remains in use for ball, car-car, and wall collisions as before.

## Parameters to Tune

| Parameter | Description | Starting point |
|-----------|-------------|---------------|
| `rest_length` | Natural length of suspension ray | ~0.4m |
| `k` (spring constant) | Stiffness — higher = bouncier | ~500 N/m |
| `c` (damping constant) | Oscillation damping — higher = sluggish | ~50 N·s/m |
| `ray_origin[4]` | Mount points relative to car center | car corners |

Tune `k` and `c` together: underdamped (`c` too low) causes bouncing, overdamped
(`c` too high) feels sluggish. Critical damping: `c = 2 * sqrt(k * mass)`.

## Implementation Sketch

```cpp
struct WheelRay {
    glm::vec3 local_offset;  // mount point relative to car center
    float rest_length;
};

void ApplySuspension(Box& car, const std::array<WheelRay, 4>& wheels,
                     float k, float c, float dt) {
    for (const auto& wheel : wheels) {
        glm::vec3 world_origin = car.position + car.rotation * wheel.local_offset;
        glm::vec3 ray_dir = car.rotation * glm::vec3(0, -1, 0);  // downward in car space

        float hit_distance = RaycastGround(world_origin, ray_dir);  // returns inf if no hit
        if (hit_distance >= wheel.rest_length) continue;  // wheel in the air

        float compression = wheel.rest_length - hit_distance;
        glm::vec3 vel_at_mount = car.velocity + glm::cross(car.angular_velocity, wheel.local_offset);
        float vel_along_ray = glm::dot(vel_at_mount, ray_dir);

        float force_mag = k * compression - c * vel_along_ray;
        force_mag = std::max(0.0f, force_mag);  // suspension can push, not pull

        glm::vec3 force = -ray_dir * force_mag;  // upward
        // Apply force + torque to car body
        car.velocity += force * car.mass_inv * dt;
        car.angular_velocity += car.inertia_tensor_inv * glm::cross(wheel.local_offset, force) * dt;
    }
}
```

## Open Questions

- Does the ball need to interact with the suspension (rolling under the car)?
  Probably not — the car body box handles ball contact.
- How does this interact with the car being airborne? Rays miss the ground,
  forces are zero, car is in free fall — correct behavior.
- Jumping: a jump impulse is applied directly to `car.velocity`, bypassing
  suspension entirely. Suspension naturally extends as the car leaves the ground.
