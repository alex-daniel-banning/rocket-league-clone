#pragma once

#include <engine/physics/Box.hpp>
#include <engine/physics/Contact.hpp>
#include <engine/physics/Plane.hpp>
#include <engine/physics/Sphere.hpp>

namespace engine::physics
{

class Collisions
{
  public:
    static bool collides(const Plane &plane, Sphere sphere);
    bool computeContact(const Box &box, const Sphere &sphere, Contact &out);
    static void handleElasticCollision(const Plane &plane, Sphere &sphere);
    static void handleElasticCollision(Box &box, Sphere &sphere);
    static float distanceSquared(const Plane &plane, Sphere sphere);
};

} // namespace engine::physics
