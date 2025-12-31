#include <engine/physics/Collisions.hpp>
#include <iostream>

namespace engine::physics
{

bool Collisions::collides(const Plane &plane, Sphere sphere)
{
    return Collisions::distanceSquared(plane, sphere) < sphere.radius * sphere.radius;
};

void Collisions::handleElasticCollision(const Plane &plane, Sphere &sphere)
{
    if (!collides(plane, sphere))
    {
        return;
    }
    glm::vec3 vPerpendicular = glm::dot(sphere.velocity, plane.getNormal()) * plane.getNormal();
    glm::vec3 vParallel      = sphere.velocity - vPerpendicular;
    // TODO, penetration correction
    if (glm::length(vPerpendicular) < 0.0001f)
    {
        float distanceFromPlane = glm::sqrt(Collisions::distanceSquared(plane, sphere));
        glm::vec3 closest;
        closest.x       = glm::clamp(sphere.position.x, plane.getMin().x, plane.getMax().x);
        closest.y       = glm::clamp(sphere.position.y, plane.getMin().y, plane.getMax().y);
        closest.z       = glm::clamp(sphere.position.z, plane.getMin().z, plane.getMax().z);
        sphere.position = closest + (sphere.radius * plane.getNormal());
        return;
    }
    sphere.velocity = vParallel - vPerpendicular;
}

void Collisions::handleElasticCollision(const Box &box, Sphere &sphere)
{
    glm::vec3 v_sphereCenterToBoxCenter = sphere.position - box.position;

    glm::vec3 closestPoint = box.position;
    // TODO, rotation -> each axis should be multiplied by the box's rotation
    std::array<glm::vec3, 3> obbAxes = {glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                                        glm::vec3(0.0f, 0.0f, 1.0f)};
    std::array<float, 3> halves      = {box.size.x / 2, box.size.y / 2, box.size.z / 2};
    for (unsigned int i = 0; i < 3; i++)
    {
        float distance = glm::dot(v_sphereCenterToBoxCenter, obbAxes[i]);
        distance       = glm::clamp(distance, -halves[i], halves[i]);
        closestPoint   = closestPoint + (distance * obbAxes[i]);
    }
    glm::vec3 v_box_surface_to_sphere_center = sphere.position - closestPoint;
    float distanceSquared =
        glm::dot(v_box_surface_to_sphere_center, v_box_surface_to_sphere_center);
    bool collides = distanceSquared < sphere.radius * sphere.radius;
    if (!collides)
    {
        return;
    }

    // TODO, handle tunneling
    // TODO, penetration correction (do for planes too?)
    glm::vec3 normal         = glm::normalize(v_box_surface_to_sphere_center);
    glm::vec3 vPerpendicular = glm::dot(sphere.velocity, normal) * normal;
    glm::vec3 vParallel      = sphere.velocity - vPerpendicular;
    if (glm::length(vPerpendicular) < 0.0001f)
    {
        // TODO - For this and the same thing for the plane function, if a sphere is going
        // pseudo-parallel into the surface, this will trigger repeatedly. Maybe it is better to set
        // the perpendicular velocity of the sphere to 0. This logic is probably different for
        // sphere v. static plane and sphere v. non-static box.
        sphere.position = closestPoint + (sphere.radius * normal);
        return;
    }
    sphere.velocity = vParallel - vPerpendicular;
}

float Collisions::distanceSquared(const Plane &plane, Sphere sphere)
{
    // find closes point on the aabb to the sphere center
    glm::vec3 closest;
    closest.x = glm::clamp(sphere.position.x, plane.getMin().x, plane.getMax().x);
    closest.y = glm::clamp(sphere.position.y, plane.getMin().y, plane.getMax().y);
    closest.z = glm::clamp(sphere.position.z, plane.getMin().z, plane.getMax().z);

    float distanceSquared = glm::dot(closest - sphere.position, closest - sphere.position);
    return distanceSquared;
};

} // namespace engine::physics
