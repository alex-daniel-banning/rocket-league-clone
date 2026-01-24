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
    static bool computeContact(const Box &box, const Sphere &sphere, Contact &out);
    static void handleElasticCollision(const Plane &plane, Sphere &sphere);
    static void resolveCollision(Box &box, Sphere &sphere, Contact contact,
                                 float coefficient_of_restitution);
    static void resolveElasticCollision(Box &box, Sphere &sphere, Contact contact);
    static float distanceSquared(const Plane &plane, Sphere sphere);
};

} // namespace engine::physics
