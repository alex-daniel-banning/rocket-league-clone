#pragma once

#include <engine/physics/Plane.hpp>
#include <engine/physics/Sphere.hpp>

namespace engine::physics
{

class Collisions
{
  public:
    static bool collides(const Plane &plane, Sphere sphere);
    static void handleElasticCollision(const Plane &plane, Sphere &sphere);
    static float distanceSquared(const Plane &plane, Sphere sphere);
};

} // namespace engine::physics
