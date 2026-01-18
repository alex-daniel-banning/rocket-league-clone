#include <engine/physics/Collisions.hpp>
#include <iostream>

namespace engine::physics
{

// TODO, pass in Contact struct
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

bool Collisions::computeContact(const Box &box, const Sphere &sphere, Contact &out)
{
    glm::vec3 v_sphereCenterToBoxCenter = sphere.position - box.position;

    glm::vec3 closestPoint = box.position;
    // TODO, rotation -> each axis should be multiplied by the box's rotation
    // clang-format off
    std::array<glm::vec3, 3> obbAxes = {
        box.rotation * glm::vec3(1.0f, 0.0f, 0.0f),
        box.rotation * glm::vec3(0.0f, 1.0f, 0.0f),
        box.rotation * glm::vec3(0.0f, 0.0f, 1.0f)
    };
    // clang-format on
    std::array<float, 3> halves = {box.size.x / 2, box.size.y / 2, box.size.z / 2};
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
        return false;
    }
    out.normal      = glm::normalize(v_box_surface_to_sphere_center);
    out.penetration = sphere.radius - std::sqrt(distanceSquared);
    out.point       = closestPoint;
    return true;
};

void Collisions::resolveElasticCollision(Box &box, Sphere &sphere, Contact contact)
{

    // new
    // This is all from the perspective of the sphere
    glm::vec3 omega       = box.angular_velocity;
    glm::vec3 r           = contact.point - box.position;
    glm::vec3 v_rel       = sphere.velocity - (box.velocity + glm::cross(omega, r));
    glm::vec3 n           = contact.normal;
    glm::mat3 rot         = glm::toMat3(box.rotation);
    glm::mat3 i_world_inv = rot * box.inertia_tensor * glm::transpose(rot);
    // 1. Relative velocity along normal
    float rel_vel_along_normal = glm::dot(v_rel, n);

    // 2. Linear mass component
    float mass_component = (1.0f / sphere.mass) + (1.0f / box.mass);

    // 3. Angular mass component
    glm::vec3 r_cross_n          = glm::cross(r, n);
    glm::vec3 angular_contrib    = glm::cross(i_world_inv * r_cross_n, r);
    float angular_mass_component = glm::dot(n, angular_contrib);

    // 4. Total effective mass along normal
    float effective_mass = mass_component + angular_mass_component;

    // 5. Impulse scalar (perfectly elastic)
    float impulse_scalar = -2.0f * rel_vel_along_normal / effective_mass;
    // clang-format off
    // TODO cleanup
    //float impulse_scalar = (-2 * glm::dot(v_rel, n))
    //                        /
    //                        ((1.0f/sphere.mass) + (1.0f/box.mass) + glm::dot(n, glm::cross(i_world_inv * glm::cross(r,n),r)));
    // clang-format on

    glm::vec3 impulse_vector = impulse_scalar * n;
    // new

    // TODO, handle tunneling
    // TODO, penetration correction (do for planes too?)
    glm::vec3 v_perpendicular_to_surface =
        glm::dot(sphere.velocity, contact.normal) * contact.normal;
    glm::vec3 v_parallel_to_surface = sphere.velocity - v_perpendicular_to_surface;
    if (glm::length(v_perpendicular_to_surface) < 0.0001f)
    {
        // TODO - For this and the same thing for the plane function, if a sphere is going
        // pseudo-parallel into the surface, this will trigger repeatedly. Maybe it is better to set
        // the perpendicular velocity of the sphere to 0. This logic is probably different for
        // sphere v. static plane and sphere v. non-static box.
        sphere.position = contact.point + (sphere.radius * contact.normal);
        return;
    }

    float vel_box_normal    = glm::dot(box.velocity, contact.normal);    // v_box_normal
    float vel_sphere_normal = glm::dot(sphere.velocity, contact.normal); // v_sphere_normal
    float vel_box_normal_prime =
        (((box.mass - sphere.mass) * vel_box_normal) + (2 * sphere.mass * vel_sphere_normal)) /
        (box.mass + sphere.mass);
    float vel_sphere_normal_prime =
        (((sphere.mass - box.mass) * vel_sphere_normal) + (2 * box.mass * vel_box_normal)) /
        (box.mass + sphere.mass);
    glm::vec3 vel_box_tangent    = box.velocity - (vel_box_normal * contact.normal);
    glm::vec3 vel_sphere_tangent = sphere.velocity - (vel_sphere_normal * contact.normal);
    box.velocity                 = vel_box_tangent + (vel_box_normal_prime * contact.normal);
    sphere.velocity              = vel_sphere_tangent + (vel_sphere_normal_prime * contact.normal);
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
