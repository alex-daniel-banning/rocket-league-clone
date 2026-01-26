#include <Print.hpp>
#include <engine/physics/collisions.hpp>

namespace engine::physics {

// TODO, pass in Contact struct
bool Collisions::Collides(const Plane& plane, Sphere sphere) {
  return Collisions::DistanceSquared(plane, sphere) <
         sphere.radius * sphere.radius;
};

void Collisions::HandleElasticCollision(const Plane& plane, Sphere& sphere) {
  if (!Collides(plane, sphere)) {
    return;
  }
  glm::vec3 v_perpendicular =
      glm::dot(sphere.velocity, plane.GetNormal()) * plane.GetNormal();
  glm::vec3 v_parallel = sphere.velocity - v_perpendicular;
  // TODO, penetration correction
  if (glm::length(v_perpendicular) < 0.0001f) {
    float distance_from_plane =
        glm::sqrt(Collisions::DistanceSquared(plane, sphere));
    glm::vec3 closest;
    closest.x =
        glm::clamp(sphere.position.x, plane.GetMin().x, plane.GetMax().x);
    closest.y =
        glm::clamp(sphere.position.y, plane.GetMin().y, plane.GetMax().y);
    closest.z =
        glm::clamp(sphere.position.z, plane.GetMin().z, plane.GetMax().z);
    sphere.position = closest + (sphere.radius * plane.GetNormal());
    return;
  }
  sphere.velocity = v_parallel - v_perpendicular;
}

float Collisions::DistanceSquared(const Plane& plane, Sphere sphere) {
  // find closes point on the aabb to the sphere center
  glm::vec3 closest;
  closest.x = glm::clamp(sphere.position.x, plane.GetMin().x, plane.GetMax().x);
  closest.y = glm::clamp(sphere.position.y, plane.GetMin().y, plane.GetMax().y);
  closest.z = glm::clamp(sphere.position.z, plane.GetMin().z, plane.GetMax().z);

  float distance_squared =
      glm::dot(closest - sphere.position, closest - sphere.position);
  return distance_squared;
};

bool Collisions::ComputeContact(const Box& box, const Sphere& sphere,
                                Contact& out) {
  glm::vec3 sphere_to_box = sphere.position - box.position;

  glm::vec3 closest_point = box.position;
  std::array<glm::vec3, 3> obb_axes = {
      box.rotation * glm::vec3(1.0f, 0.0f, 0.0f),
      box.rotation * glm::vec3(0.0f, 1.0f, 0.0f),
      box.rotation * glm::vec3(0.0f, 0.0f, 1.0f)};
  std::array<float, 3> halves = {box.size.x / 2, box.size.y / 2,
                                 box.size.z / 2};
  for (unsigned int i = 0; i < 3; i++) {
    float distance = glm::dot(sphere_to_box, obb_axes[i]);
    distance = glm::clamp(distance, -halves[i], halves[i]);
    closest_point = closest_point + (distance * obb_axes[i]);
  }
  glm::vec3 v_box_surface_to_sphere_center = sphere.position - closest_point;
  float distance_squared =
      glm::dot(v_box_surface_to_sphere_center, v_box_surface_to_sphere_center);
  bool collides = distance_squared < sphere.radius * sphere.radius;
  if (!collides) {
    return false;
  }
  out.normal = glm::normalize(v_box_surface_to_sphere_center);
  out.penetration = sphere.radius - std::sqrt(distance_squared);
  out.point = closest_point;
  return true;
};

void Collisions::ResolveCollision(Box& box, Sphere& sphere, Contact contact,
                                  float coefficient_of_restitution) {
  // TODO, handle tunneling
  const float separation_slop = 1e-3f;

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
    float total_correction = contact.penetration + separation_slop;
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

void Collisions::ResolveElasticCollision(Box& box, Sphere& sphere,
                                         Contact contact) {
  ResolveCollision(box, sphere, contact, 1.0f);
}

bool Collisions::ComputeContact(const Box& box_a, const Box& box_b,
                                Contact& out) {
  const float epsilon = 1e-6f;
  float minimum_penetration = std::numeric_limits<float>::max();
  glm::vec3 penetration_axis;

  // Extract axes from both boxes
  const std::array<glm::vec3, 3> axes_a = GetAxesFromQuaternion(box_a.rotation);
  const std::array<glm::vec3, 3> axes_b = GetAxesFromQuaternion(box_b.rotation);

  std::vector<glm::vec3> axes_to_test;
  axes_to_test.reserve(15);
  axes_to_test.insert(axes_to_test.end(), axes_a.begin(), axes_a.end());
  axes_to_test.insert(axes_to_test.end(), axes_b.begin(), axes_b.end());
  for (const glm::vec3 edge_a : axes_a) {
    for (const glm::vec3 edge_b : axes_b) {
      glm::vec3 axis = glm::cross(edge_a, edge_b);
      if (glm::length(axis) >= epsilon)
        axes_to_test.push_back(glm::normalize(axis));
    }
  }

  for (glm::vec3 axis : axes_to_test) {
    float overlap = CalculateOverlap(axis, box_a, box_b, axes_a, axes_b);

    // If there's an axis with no overlap, it means the boxes are separated.
    if (overlap <= 0) return false;

    if (overlap < minimum_penetration) {
      minimum_penetration = overlap;
      penetration_axis = axis;
      // Ensure axis points from A to B
      if (glm::dot(box_b.position - box_a.position, axis) < 0)
        penetration_axis = -axis;
    }
  }

  // Find support point on boxB in direction of penetration normal
  glm::vec3 support_b = box_b.position;
  const std::array<float, 3> half_extents_b = {
      box_b.size.x * 0.5f, box_b.size.y * 0.5f, box_b.size.z * 0.5f};
  for (int i = 0; i < 3; i++) {
    float projection = glm::dot(axes_b[i], penetration_axis);
    support_b +=
        axes_b[i] * half_extents_b[i] * (projection > 0 ? 1.0f : -1.0f);
  }

  // Find support point on boxA in opposite direction
  glm::vec3 support_a = box_a.position;
  const std::array<float, 3> half_extents_a = {
      box_a.size.x * 0.5f, box_a.size.y * 0.5f, box_a.size.z * 0.5f};
  for (int i = 0; i < 3; i++) {
    float projection = glm::dot(axes_a[i], penetration_axis);
    support_a +=
        axes_a[i] * half_extents_a[i] * (projection > 0 ? 1.0f : -1.0f);
  }

  out.penetration = minimum_penetration;
  out.normal = penetration_axis;
  out.point = (support_a + support_b) * 0.5f;
  return true;
}
void Collisions::ResolveCollision(Box& box_a, Box& box_b, Contact contact,
                                  float coefficient_of_restitution) { /*
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
       -(1 + coefficient_of_restitution) * rel_vel_along_normal /
   effective_mass; glm::vec3 impulse_vector = impulse_scalar * n;

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
 */
}

std::array<glm::vec3, 3> Collisions::GetAxesFromQuaternion(
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

float Collisions::CalculateOverlap(glm::vec3 axis, const Box& box_a,
                                   const Box& box_b,
                                   const std::array<glm::vec3, 3> axes_a,
                                   const std::array<glm::vec3, 3> axes_b) {
  // Project box centers onto axis
  float proj_a = glm::dot(box_a.position, axis);
  float proj_b = glm::dot(box_b.position, axis);

  // Calculate extents (half-widths of projections)
  float r_a = std::abs(glm::dot(axes_a[0], axis)) * box_a.size.x * 0.5f +
              std::abs(glm::dot(axes_a[1], axis)) * box_a.size.y * 0.5f +
              std::abs(glm::dot(axes_a[2], axis)) * box_a.size.z * 0.5f;

  float r_b = std::abs(glm::dot(axes_b[0], axis)) * box_b.size.x * 0.5f +
              std::abs(glm::dot(axes_b[1], axis)) * box_b.size.y * 0.5f +
              std::abs(glm::dot(axes_b[2], axis)) * box_b.size.z * 0.5f;

  float distance = std::abs(proj_a - proj_b);
  return (r_a + r_b) - distance;
}

}  // namespace engine::physics
