#include <engine/physics/Collisions.hpp>

namespace engine::physics
{

bool Collisions::collides(const Plane &plane, Sphere sphere)
{
    return Collisions::distanceSquared(plane, sphere) < sphere.radius * sphere.radius;
};

void Collisions::handleElasticCollision(const Plane &plane, Sphere &sphere)
{
    glm::vec3 vPerpendicular = glm::dot(sphere.velocity, plane.getNormal()) * plane.getNormal();
    glm::vec3 vParallel      = sphere.velocity - vPerpendicular;
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
