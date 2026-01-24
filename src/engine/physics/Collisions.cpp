#include <engine/physics/Collisions.hpp>

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
    // clang-format off
    std::array<glm::vec3, 3> obbAxes = {
        box.rotation * glm::vec3(1.0f, 0.0f, 0.0f),
        box.rotation * glm::vec3(0.0f, 1.0f, 0.0f),
        box.rotation * glm::vec3(0.0f, 0.0f, 1.0f)
    }; // clang-format on
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
    resolveCollision(box, sphere, contact, 1.0f);
}

void Collisions::resolveCollision(Box &box, Sphere &sphere, Contact contact,
                                  float coefficient_of_restitution)
{
    // TODO, handle tunneling
    const float SEPARATION_SLOP = 1e-3f;

    // This is all from the perspective of the sphere
    glm::vec3 omega            = box.angular_velocity;
    glm::vec3 r                = contact.point - box.position;
    glm::vec3 v_rel            = sphere.velocity - (box.velocity + glm::cross(omega, r));
    glm::vec3 n                = contact.normal;
    glm::mat3 rot              = glm::toMat3(box.rotation);
    glm::mat3 i_world_inv      = rot * box.inertia_tensor_inv * glm::transpose(rot);
    float rel_vel_along_normal = glm::dot(v_rel, n);

    // We should never have a situation where a sphere impacts a box, while also moving farther from
    // it. If we end up running into this, figure out what's leading to it and how to handle it.
    assert(rel_vel_along_normal <= 0.0f);

    // Linear mass component
    float mass_component = (1.0f / sphere.mass) + (1.0f / box.mass);

    // Angular mass component
    glm::vec3 r_cross_n          = glm::cross(r, n);
    glm::vec3 angular_contrib    = glm::cross(i_world_inv * r_cross_n, r);
    float angular_mass_component = glm::dot(n, angular_contrib);

    // Total effective mass along normal
    float effective_mass = mass_component + angular_mass_component;

    float impulse_scalar =
        -(1 + coefficient_of_restitution) * rel_vel_along_normal / effective_mass;
    glm::vec3 impulse_vector = impulse_scalar * n;

    // Apply the impulse
    sphere.velocity += impulse_vector / sphere.mass;
    box.velocity -= impulse_vector / box.mass;
    box.angular_velocity += i_world_inv * glm::cross(r, -impulse_vector);

    // Penetration correction
    if (contact.penetration > 0.0f)
    {
        // Separate the two along normal
        float total_correction  = contact.penetration + SEPARATION_SLOP;
        float total_inv_mass    = (1.0f / sphere.mass) + (1.0f / box.mass);
        float sphere_correction = (total_correction / total_inv_mass) * (1.0f / sphere.mass);
        float box_correction    = (total_correction / total_inv_mass) * (1.0f / box.mass);
        sphere.position += contact.normal * sphere_correction;
        box.position -= contact.normal * box_correction;
    }
    return;
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
