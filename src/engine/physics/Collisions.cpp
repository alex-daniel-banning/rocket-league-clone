#include <Print.hpp>
#include <engine/physics/Collisions.hpp>

namespace engine::physics {

// TODO, pass in Contact struct
bool Collisions::collides(const Plane& plane, Sphere sphere) {
  return Collisions::distanceSquared(plane, sphere) <
         sphere.radius * sphere.radius;
};

void Collisions::handleElasticCollision(const Plane& plane, Sphere& sphere) {
  if (!collides(plane, sphere)) {
    return;
  }
  glm::vec3 vPerpendicular =
      glm::dot(sphere.velocity, plane.getNormal()) * plane.getNormal();
  glm::vec3 vParallel = sphere.velocity - vPerpendicular;
  // TODO, penetration correction
  if (glm::length(vPerpendicular) < 0.0001f) {
    float distanceFromPlane =
        glm::sqrt(Collisions::distanceSquared(plane, sphere));
    glm::vec3 closest;
    closest.x =
        glm::clamp(sphere.position.x, plane.getMin().x, plane.getMax().x);
    closest.y =
        glm::clamp(sphere.position.y, plane.getMin().y, plane.getMax().y);
    closest.z =
        glm::clamp(sphere.position.z, plane.getMin().z, plane.getMax().z);
    sphere.position = closest + (sphere.radius * plane.getNormal());
    return;
  }
  sphere.velocity = vParallel - vPerpendicular;
}

float Collisions::distanceSquared(const Plane& plane, Sphere sphere) {
  // find closes point on the aabb to the sphere center
  glm::vec3 closest;
  closest.x = glm::clamp(sphere.position.x, plane.getMin().x, plane.getMax().x);
  closest.y = glm::clamp(sphere.position.y, plane.getMin().y, plane.getMax().y);
  closest.z = glm::clamp(sphere.position.z, plane.getMin().z, plane.getMax().z);

  float distanceSquared =
      glm::dot(closest - sphere.position, closest - sphere.position);
  return distanceSquared;
};

bool Collisions::computeContact(const Box& box, const Sphere& sphere,
                                Contact& out) {
  glm::vec3 v_sphereCenterToBoxCenter = sphere.position - box.position;

  glm::vec3 closestPoint = box.position;
  std::array<glm::vec3, 3> obbAxes = {
      box.rotation * glm::vec3(1.0f, 0.0f, 0.0f),
      box.rotation * glm::vec3(0.0f, 1.0f, 0.0f),
      box.rotation * glm::vec3(0.0f, 0.0f, 1.0f)};
  std::array<float, 3> halves = {box.size.x / 2, box.size.y / 2,
                                 box.size.z / 2};
  for (unsigned int i = 0; i < 3; i++) {
    float distance = glm::dot(v_sphereCenterToBoxCenter, obbAxes[i]);
    distance = glm::clamp(distance, -halves[i], halves[i]);
    closestPoint = closestPoint + (distance * obbAxes[i]);
  }
  glm::vec3 v_box_surface_to_sphere_center = sphere.position - closestPoint;
  float distanceSquared =
      glm::dot(v_box_surface_to_sphere_center, v_box_surface_to_sphere_center);
  bool collides = distanceSquared < sphere.radius * sphere.radius;
  if (!collides) {
    return false;
  }
  out.normal = glm::normalize(v_box_surface_to_sphere_center);
  out.penetration = sphere.radius - std::sqrt(distanceSquared);
  out.point = closestPoint;
  return true;
};

void Collisions::resolveCollision(Box& box, Sphere& sphere, Contact contact,
                                  float coefficient_of_restitution) {
  // TODO, handle tunneling
  const float SEPARATION_SLOP = 1e-3f;

  // This is all from the perspective of the sphere
  glm::vec3 omega = box.angular_velocity;
  glm::vec3 r = contact.point - box.position;
  glm::vec3 v_rel = sphere.velocity - (box.velocity + glm::cross(omega, r));
  glm::vec3 n = contact.normal;
  glm::mat3 rot = glm::toMat3(box.rotation);
  glm::mat3 i_world_inv = rot * box.inertia_tensor_inv * glm::transpose(rot);
  float rel_vel_along_normal = glm::dot(v_rel, n);

  // We should never have a situation where a sphere impacts a box, while also
  // moving farther from it. If we end up running into this, figure out what's
  // leading to it and how to handle it.
  assert(rel_vel_along_normal <= 0.0f);

  // Linear mass component
  float mass_component = (1.0f / sphere.mass) + (1.0f / box.mass);

  // Angular mass component
  glm::vec3 r_cross_n = glm::cross(r, n);
  glm::vec3 angular_contrib = glm::cross(i_world_inv * r_cross_n, r);
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
  if (contact.penetration > 0.0f) {
    // Separate the two along normal
    float total_correction = contact.penetration + SEPARATION_SLOP;
    float total_inv_mass = (1.0f / sphere.mass) + (1.0f / box.mass);
    float sphere_correction =
        (total_correction / total_inv_mass) * (1.0f / sphere.mass);
    float box_correction =
        (total_correction / total_inv_mass) * (1.0f / box.mass);
    sphere.position += contact.normal * sphere_correction;
    box.position -= contact.normal * box_correction;
  }
  return;
}

void Collisions::resolveElasticCollision(Box& box, Sphere& sphere,
                                         Contact contact) {
  resolveCollision(box, sphere, contact, 1.0f);
}

bool Collisions::computeContact(const Box& boxA, const Box& boxB,
                                Contact& out) {
  const float epsilon = 1e-6f;
  float minimum_penetration = std::numeric_limits<float>::max();
  glm::vec3 penetration_axis;

  // Extract axes from both boxes
  const std::array<glm::vec3, 3> axesA = getAxesFromQuaternion(boxA.rotation);
  const std::array<glm::vec3, 3> axesB = getAxesFromQuaternion(boxB.rotation);

  std::vector<glm::vec3> axes_to_test;
  axes_to_test.reserve(15);
  axes_to_test.insert(axes_to_test.end(), axesA.begin(), axesA.end());
  axes_to_test.insert(axes_to_test.end(), axesB.begin(), axesB.end());
  for (const glm::vec3 edgeA : axesA) {
    for (const glm::vec3 edgeB : axesB) {
      glm::vec3 axis = glm::cross(edgeA, edgeB);
      if (glm::length(axis) >= epsilon)
        axes_to_test.push_back(glm::normalize(axis));
    }
  }

  for (glm::vec3 axis : axes_to_test) {
    float overlap = calculate_overlap(axis, boxA, boxB, axesA, axesB);

    // If there's an axis with no overlap, it means the boxes are separated.
    if (overlap <= 0) return false;

    if (overlap < minimum_penetration) {
      minimum_penetration = overlap;
      penetration_axis = axis;
      // Ensure axis points from A to B
      if (glm::dot(boxB.position - boxA.position, axis) < 0)
        penetration_axis = -axis;
    }
  }

  // Find support point on boxB in direction of penetration normal
  glm::vec3 supportB = boxB.position;
  const std::array<float, 3> halfExtentsB = {
      boxB.size.x * 0.5f, boxB.size.y * 0.5f, boxB.size.z * 0.5f};
  for (int i = 0; i < 3; i++) {
    float projection = glm::dot(axesB[i], penetration_axis);
    supportB += axesB[i] * halfExtentsB[i] * (projection > 0 ? 1.0f : -1.0f);
  }

  // Find support point on boxA in opposite direction
  glm::vec3 supportA = boxA.position;
  const std::array<float, 3> halfExtentsA = {
      boxA.size.x * 0.5f, boxA.size.y * 0.5f, boxA.size.z * 0.5f};
  for (int i = 0; i < 3; i++) {
    float projection = glm::dot(axesA[i], penetration_axis);
    supportA += axesA[i] * halfExtentsA[i] * (projection > 0 ? 1.0f : -1.0f);
  }

  out.penetration = minimum_penetration;
  out.normal = penetration_axis;
  out.point = (supportA + supportB) * 0.5f;
  return true;
}
void Collisions::ResolveCollision(Box& box_a, Box& box_b, Contact contact,
                                  float coefficient_of_restitution) {
  // TODO, handle tunneling
  const float kSeparationSlop = 1e-3f;

  // This is all from the perspective of the sphere
  glm::vec3 omega = box.angular_velocity;
  glm::vec3 r = contact.point - box.position;
  glm::vec3 v_rel = sphere.velocity - (box.velocity + glm::cross(omega, r));
  glm::vec3 n = contact.normal;
  glm::mat3 rot = glm::toMat3(box.rotation);
  glm::mat3 i_world_inv = rot * box.inertia_tensor_inv * glm::transpose(rot);
  float rel_vel_along_normal = glm::dot(v_rel, n);

  // We should never have a situation where a sphere impacts a box, while also
  // moving farther from it. If we end up running into this, figure out what's
  // leading to it and how to handle it.
  assert(rel_vel_along_normal <= 0.0f);

  // Linear mass component
  float mass_component = (1.0f / sphere.mass) + (1.0f / box.mass);

  // Angular mass component
  glm::vec3 r_cross_n = glm::cross(r, n);
  glm::vec3 angular_contrib = glm::cross(i_world_inv * r_cross_n, r);
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
  if (contact.penetration > 0.0f) {
    // Separate the two along normal
    float total_correction = contact.penetration + kSeparationSlop;
    float total_inv_mass = (1.0f / sphere.mass) + (1.0f / box.mass);
    float sphere_correction =
        (total_correction / total_inv_mass) * (1.0f / sphere.mass);
    float box_correction =
        (total_correction / total_inv_mass) * (1.0f / box.mass);
    sphere.position += contact.normal * sphere_correction;
    box.position -= contact.normal * box_correction;
  }
  return;
}

std::array<glm::vec3, 3> Collisions::getAxesFromQuaternion(
    glm::quat q) {  // clang-format off
  // Right axis (local X)
  glm::vec3 right = glm::vec3(
    1 - 2 * (q.y * q.y + q.z * q.z),
    2 * (q.x * q.y + q.w * q.z),
    2 * (q.x * q.z - q.w * q.y)
  );

  // Up axis (local Y)
  glm::vec3 up = glm::vec3(
    2 * (q.x * q.y - q.w * q.z),
    1 - 2 * (q.x * q.x + q.z * q.z),
    2 * (q.y * q.z + q.w * q.x)
  );

  // Forward axis (local Z)
  glm::vec3 forward = glm::vec3(
    2 * (q.x * q.z + q.w * q.y),
    2 * (q.y * q.z - q.w * q.x),
    1 - 2 * (q.x * q.x + q.y * q.y)
  );

  return { right, up, forward };
}  // clang-format on

float Collisions::calculate_overlap(glm::vec3 axis, const Box& boxA,
                                    const Box& boxB,
                                    const std::array<glm::vec3, 3> axesA,
                                    const std::array<glm::vec3, 3> axesB) {
  // Project box centers onto axis
  float projA = glm::dot(boxA.position, axis);
  float projB = glm::dot(boxB.position, axis);

  // Calculate extents (half-widths of projections)
  float rA = std::abs(glm::dot(axesA[0], axis)) * boxA.size.x * 0.5f +
             std::abs(glm::dot(axesA[1], axis)) * boxA.size.y * 0.5f +
             std::abs(glm::dot(axesA[2], axis)) * boxA.size.z * 0.5f;

  float rB = std::abs(glm::dot(axesB[0], axis)) * boxB.size.x * 0.5f +
             std::abs(glm::dot(axesB[1], axis)) * boxB.size.y * 0.5f +
             std::abs(glm::dot(axesB[2], axis)) * boxB.size.z * 0.5f;

  float distance = std::abs(projA - projB);
  return (rA + rB) - distance;
}

}  // namespace engine::physics
