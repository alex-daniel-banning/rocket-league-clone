#include <Print.hpp>
#include <engine/physics/collision_type.hpp>
#include <engine/physics/collisions.hpp>

namespace {

struct AxisClassification {
  std::vector<int> max_aligned;
  std::vector<int> other;
};

AxisClassification ClassifyAxesByAlignment(const std::array<glm::vec3, 3>& axes,
                                           const glm::vec3& penetration_axis,
                                           float epsilon = 0.001f) {
  float dots[3];
  for (int i = 0; i < 3; i++) {
    dots[i] = std::abs(glm::dot(axes[i], penetration_axis));
  }

  float max_dot = std::max({dots[0], dots[1], dots[2]});

  AxisClassification result;
  for (int i = 0; i < 3; i++) {
    if (std::abs(dots[i] - max_dot) < epsilon)
      result.max_aligned.push_back(i);
    else
      result.other.push_back(i);
  }
  return result;
}

// Has axis that are perfectly mirrored across the penetration axis
bool HasSymmetricalAxis(const std::array<glm::vec3, 3> axes,
                        const glm::vec3& penetration_axis,
                        float epsilon = 0.001f) {
  return ClassifyAxesByAlignment(axes, penetration_axis, epsilon)
             .max_aligned.size() > 1;
}

int MirroredAxesCount(const std::array<glm::vec3, 3> axes,
                      const glm::vec3& penetration_axis,
                      float epsilon = 0.001f) {
  AxisClassification a =
      ClassifyAxesByAlignment(axes, penetration_axis, epsilon);
  return a.max_aligned.size() == 1 ? 0 : a.max_aligned.size();
}

void ClampSegmentToPlane(glm::vec3& v1, glm::vec3& v2,
                         const glm::vec3& plane_point,
                         const glm::vec3& plane_normal) {
  float d1 = glm::dot(v1 - plane_point, plane_normal);
  float d2 = glm::dot(v2 - plane_point, plane_normal);

  glm::vec3 orig_v1 = v1;

  // Clamp v1 to plane if outside
  if (d1 < 0) {
    float interpolation = d1 / (d1 - d2);
    v1 = orig_v1 + interpolation * (v2 - orig_v1);
  }

  // Clamp v2 to plane if outside
  if (d2 < 0) {
    float interpolation = d2 / (d2 - d1);
    v2 = v2 + interpolation * (orig_v1 - v2);
  }

  assert(v1 != v2);
}

}  // namespace

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
  std::array<float, 3> halves = {box.Size().x / 2, box.Size().y / 2,
                                 box.Size().z / 2};
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
  out.points.push_back(closest_point);
  return true;
};

void Collisions::ResolveCollision(Box& box, Sphere& sphere, Contact contact,
                                  float coefficient_of_restitution) {
  // TODO, handle tunneling
  const float separation_slop = 1e-3f;

  // This is all from the perspective of the sphere
  glm::vec3 omega = box.angular_velocity;
  glm::vec3 r = contact.points[0] - box.position;
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
  float penetration = std::numeric_limits<float>::max();
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

    if (overlap < penetration) {
      penetration = overlap;
      penetration_axis = axis;
      // Ensure axis points from A to B
      if (glm::dot(box_b.position - box_a.position, axis) < 0)
        penetration_axis = -penetration_axis;
    }
  }

  CollisionType type = DetermineCollisionType(penetration_axis, axes_a, axes_b);

  std::vector<glm::vec3> contact_points;
  switch (type) {
    case CollisionType::FACE_FACE:
      contact_points =
          ClipFaceFace(box_a, box_b, penetration_axis, axes_a, axes_b);
      break;
    case CollisionType::EDGE_FACE:
      contact_points =
          MirroredAxesCount(axes_a, penetration_axis) > 1
              ? ClipEdgeToFace(box_a, box_b, penetration_axis, axes_a, axes_b)
              : ClipEdgeToFace(box_b, box_a, penetration_axis, axes_b, axes_a);
      break;
    case CollisionType::EDGE_EDGE:
      contact_points =
          ClipEdgeEdge(box_a, box_b, penetration_axis, axes_a, axes_b);
      break;
    case CollisionType::CORNER_FACE:
      contact_points =
          MirroredAxesCount(axes_a, penetration_axis) == 3
              ? ClipCornerToFace(box_a, box_b, penetration_axis, axes_a, axes_b)
              : ClipCornerToFace(box_b, box_a, penetration_axis, axes_b,
                                 axes_a);
      break;
    case CollisionType::CORNER_EDGE:
      // TODO
      break;
    case CollisionType::CORNER_CORNER:
      // TODO
      break;
  }

  for (auto& p : contact_points) {
    p = p + (0.5f * penetration * penetration_axis);
  }
  out.points.insert(out.points.end(), contact_points.begin(),
                    contact_points.end());
  out.normal = penetration_axis;
  out.penetration = penetration;
  assert(out.points.size() > 0);
  return true;
}

void Collisions::ResolveCollision(Box& box_a, Box& box_b, Contact contact,
                                  float coefficient_of_restitution) {
  // TODO, handle tunneling
  const float separation_slop = 1e-3f;

  for (glm::vec3 point : contact.points) {
    // TODO, resolve collision when there are multiple contact points
    // Contact point relative to each body's center of mass
    glm::vec3 r_a = point - box_a.position;
    glm::vec3 r_b = point - box_b.position;

    // Velocity at contact point for each body
    glm::vec3 v_contact_a =
        box_a.velocity + glm::cross(box_a.angular_velocity, r_a);
    glm::vec3 v_contact_b =
        box_b.velocity + glm::cross(box_b.angular_velocity, r_b);
    glm::vec3 rel_vel = v_contact_a - v_contact_b;

    // Transform inertia tensors to world space
    glm::mat3 rot_a = glm::toMat3(box_a.rotation);
    glm::mat3 rot_b = glm::toMat3(box_b.rotation);
    glm::mat3 i_world_inv_a =
        rot_a * box_a.inertia_tensor_inv * glm::transpose(rot_a);
    glm::mat3 i_world_inv_b =
        rot_b * box_b.inertia_tensor_inv * glm::transpose(rot_b);

    // Compute effective mass along collision normal
    glm::vec3 r_cross_n_a = glm::cross(r_a, contact.normal);
    glm::vec3 r_cross_n_b = glm::cross(r_b, contact.normal);
    float m_eff =
        (1 / box_a.mass) + (1 / box_b.mass) +
        glm::dot(contact.normal, glm::cross(i_world_inv_a * r_cross_n_a, r_a)) +
        glm::dot(contact.normal, glm::cross(i_world_inv_b * r_cross_n_b, r_b));

    // Impulse magnitude
    float v_n = glm::dot(rel_vel, contact.normal);
    float j = -(1.0f + coefficient_of_restitution) * v_n / m_eff;
    glm::vec3 impulse =
        (j * contact.normal) / static_cast<float>(contact.points.size());

    // Apply the impulse
    box_a.velocity += impulse / box_a.mass;
    box_b.velocity -= impulse / box_b.mass;
    box_a.angular_velocity += i_world_inv_a * glm::cross(r_a, impulse);
    box_b.angular_velocity += i_world_inv_b * glm::cross(r_b, impulse);
  }
  // Penetration correction
  if (contact.penetration > 0.0f) {
    // Separate the two along normal
    float total_correction = contact.penetration + separation_slop;
    float total_inv_mass = (1.0f / box_a.mass) + (1.0f / box_b.mass);
    float box_a_correction =
        (total_correction / total_inv_mass) * (1.0f / box_a.mass);
    float box_b_correction =
        (total_correction / total_inv_mass) * (1.0f / box_b.mass);
    box_a.position -= contact.normal * box_a_correction;
    box_b.position += contact.normal * box_b_correction;
  }
  return;
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
  return {glm::normalize(right), glm::normalize(up), glm::normalize(forward)};
}  // clang-format on

float Collisions::CalculateOverlap(glm::vec3 axis, const Box& box_a,
                                   const Box& box_b,
                                   const std::array<glm::vec3, 3> axes_a,
                                   const std::array<glm::vec3, 3> axes_b) {
  // Project box centers onto axis
  float proj_a = glm::dot(box_a.position, axis);
  float proj_b = glm::dot(box_b.position, axis);

  // Calculate extents (half-widths of projections)
  float r_a = std::abs(glm::dot(axes_a[0], axis)) * box_a.Size().x * 0.5f +
              std::abs(glm::dot(axes_a[1], axis)) * box_a.Size().y * 0.5f +
              std::abs(glm::dot(axes_a[2], axis)) * box_a.Size().z * 0.5f;

  float r_b = std::abs(glm::dot(axes_b[0], axis)) * box_b.Size().x * 0.5f +
              std::abs(glm::dot(axes_b[1], axis)) * box_b.Size().y * 0.5f +
              std::abs(glm::dot(axes_b[2], axis)) * box_b.Size().z * 0.5f;

  float distance = std::abs(proj_a - proj_b);
  return (r_a + r_b) - distance;
}

CollisionType Collisions::DetermineCollisionType(
    const glm::vec3 penetration_axis, const std::array<glm::vec3, 3>& axes_a,
    const std::array<glm::vec3, 3>& axes_b) {
  int mirrored_axes_count_a = MirroredAxesCount(axes_a, penetration_axis);
  int mirrored_axes_count_b = MirroredAxesCount(axes_b, penetration_axis);
  if (mirrored_axes_count_a == 0 && mirrored_axes_count_b == 0) {
    return CollisionType::FACE_FACE;
  }
  if (mirrored_axes_count_a == 2 && mirrored_axes_count_b == 0 ||
      mirrored_axes_count_a == 0 && mirrored_axes_count_b == 2) {
    return CollisionType::EDGE_FACE;
  }
  if (mirrored_axes_count_a == 3 && mirrored_axes_count_b == 0 ||
      mirrored_axes_count_a == 0 && mirrored_axes_count_b == 3) {
    return CollisionType::CORNER_FACE;
  }
  if (mirrored_axes_count_a == 3 && mirrored_axes_count_b == 0 ||
      mirrored_axes_count_a == 0 && mirrored_axes_count_b == 3) {
    return CollisionType::CORNER_FACE;
  }
  if (mirrored_axes_count_a == 2 && mirrored_axes_count_b == 2) {
    return CollisionType::EDGE_EDGE;
  }
  if (mirrored_axes_count_a == 3 && mirrored_axes_count_b == 2 ||
      mirrored_axes_count_a == 2 && mirrored_axes_count_b == 3) {
    return CollisionType::CORNER_EDGE;
  }
  if (mirrored_axes_count_a == 3 && mirrored_axes_count_b == 3) {
    return CollisionType::CORNER_CORNER;
  }
  return CollisionType::FACE_FACE;
}

std::vector<glm::vec3> Collisions::ClipFaceFace(
    const Box& box_ref, const Box& box_inc, glm::vec3 penetration_axis,
    const std::array<glm::vec3, 3>& axes_ref,
    const std::array<glm::vec3, 3>& axes_inc) {
  // 1. Find which box_inc axis is most aligned with penetration axis
  int inc_idx = 0;
  float inc_dot = glm::dot(axes_inc[0], penetration_axis);
  float max_abs = std::abs(inc_dot);
  for (int i = 1; i < 3; i++) {
    float d = glm::dot(axes_inc[i], penetration_axis);
    if (std::abs(d) > max_abs) {
      max_abs = std::abs(d);
      inc_dot = d;
      inc_idx = i;
    }
  }
  // After finding inc_idx, check, for a tie with second-best axis
  float inc_dots[3];
  for (int i = 0; i < 3; i++) {
    inc_dots[i] = std::abs(glm::dot(axes_inc[i], penetration_axis));
  }
  // Sort to find top two
  int first_inc = 0, second_inc = 1;
  if (inc_dots[1] > inc_dots[0]) std::swap(first_inc, second_inc);
  if (inc_dots[2] > inc_dots[first_inc]) {
    second_inc = first_inc;
    first_inc = 2;
  } else if (inc_dots[2] > inc_dots[second_inc]) {
    second_inc = 2;
  }

  // 2. Find which box_ref axis is most aligned with penetration axis
  int ref_idx = 0;
  float ref_dot = glm::dot(axes_ref[0], penetration_axis);
  max_abs = std::abs(ref_dot);
  for (int i = 1; i < 3; i++) {
    float d = glm::dot(axes_ref[i], penetration_axis);
    if (std::abs(d) > max_abs) {
      max_abs = std::abs(d);
      ref_dot = d;
      ref_idx = i;
    }
  }
  // 3. Build 4 corners of incident face
  int tangent_1 = (inc_idx + 1) % 3;
  int tangent_2 = (inc_idx + 2) % 3;

  // If the dot product of the inc_axis and penetration is negative, then the
  // inc_axis is pointing in the correct direction and doesn't need to be
  // corrected.
  float inc_sign = (inc_dot > 0.0f) ? -1.0f : 1.0f;
  glm::vec3 inc_center =
      box_inc.position +
      (inc_sign * box_inc.HalfExtents()[inc_idx] * axes_inc[inc_idx]);

  std::vector<glm::vec3> incident_verts = {
      // clang-format off
      inc_center + box_inc.HalfExtents()[tangent_1] * axes_inc[tangent_1] + box_inc.HalfExtents()[tangent_2] * axes_inc[tangent_2],
      inc_center + box_inc.HalfExtents()[tangent_1] * axes_inc[tangent_1] - box_inc.HalfExtents()[tangent_2] * axes_inc[tangent_2],
      inc_center - box_inc.HalfExtents()[tangent_1] * axes_inc[tangent_1] - box_inc.HalfExtents()[tangent_2] * axes_inc[tangent_2],
      inc_center - box_inc.HalfExtents()[tangent_1] * axes_inc[tangent_1] + box_inc.HalfExtents()[tangent_2] * axes_inc[tangent_2]
    };  // clang-format on

  // 4. Clip against 4 side planes of reference face
  int ref_tangent_1 = (ref_idx + 1) % 3;
  int ref_tangent_2 = (ref_idx + 2) % 3;
  // If the dot product of the inc_axis and penetration is negative, then the
  // inc_axis is pointing in the correct direction and doesn't need to be
  // corrected.
  float ref_sign = (ref_dot < 0.0f) ? -1.0f : 1.0f;
  glm::vec3 ref_center =
      box_ref.position +
      (ref_sign * box_ref.HalfExtents()[ref_idx] * axes_ref[ref_idx]);

  // clang-format off
  // Clip against + tangent_1 plane
  incident_verts = ClipPolygonAgainstPlane(incident_verts, ref_center + box_ref.HalfExtents()[ref_tangent_1] * axes_ref[ref_tangent_1], -axes_ref[ref_tangent_1]);
  // Clip against - tangent_1 plane
  incident_verts = ClipPolygonAgainstPlane(incident_verts, ref_center - box_ref.HalfExtents()[ref_tangent_1] * axes_ref[ref_tangent_1], axes_ref[ref_tangent_1]);
  // Clip against + tangent_2 plane
  incident_verts = ClipPolygonAgainstPlane(incident_verts, ref_center + box_ref.HalfExtents()[ref_tangent_2] * axes_ref[ref_tangent_2], -axes_ref[ref_tangent_2]);
  // Clip against - tangent_2 plane
  incident_verts = ClipPolygonAgainstPlane(incident_verts, ref_center - box_ref.HalfExtents()[ref_tangent_2] * axes_ref[ref_tangent_2], axes_ref[ref_tangent_2]);
  // clang-format on

  // Step 5. Keep only points behind/on reference face
  std::vector<glm::vec3> contacts;
  for (const auto& inc_vert : incident_verts) {
    float distance = glm::dot(inc_vert - ref_center, penetration_axis);
    if (distance <= 0.0f) {
      contacts.push_back(inc_vert);
    }
  }
  return contacts;
}

std::vector<glm::vec3> Collisions::ClipEdgeToFace(
    const Box& box_inc, const Box& box_ref, const glm::vec3 penetration_axis,
    const std::array<glm::vec3, 3>& axes_inc,
    const std::array<glm::vec3, 3>& axes_ref) {
  // Incident (inc) = box with the edges that are getting clipped
  // Reference (ref) = box with the planes that do the clipping

  glm::vec3 effective_penetration_axis = -penetration_axis;
  AxisClassification axis_classification =
      ClassifyAxesByAlignment(axes_inc, penetration_axis);

  assert(axis_classification.other.size() == 1);
  int along_edge = axis_classification.other[0];
  int max_aligned_1 = axis_classification.max_aligned[0];
  int max_aligned_2 = axis_classification.max_aligned[1];

  float sign1 =
      glm::dot(axes_inc[max_aligned_1], effective_penetration_axis) > 0 ? 1.0f
                                                                        : -1.0f;
  float sign2 =
      glm::dot(axes_inc[max_aligned_2], effective_penetration_axis) > 0 ? 1.0f
                                                                        : -1.0f;
  glm::vec3 edge_center =
      box_inc.position +
      sign1 * box_inc.HalfExtents()[max_aligned_1] * axes_inc[max_aligned_1] +
      sign2 * box_inc.HalfExtents()[max_aligned_2] * axes_inc[max_aligned_2];
  glm::vec3 v1 =
      edge_center + box_inc.HalfExtents()[along_edge] * axes_inc[along_edge];
  glm::vec3 v2 =
      edge_center - box_inc.HalfExtents()[along_edge] * axes_inc[along_edge];

  // Clip against reference face's 4 side planes
  // Find reference face normal (most aligned with penetration axis on box_ref)
  int face_axis = 0;
  float max_dot = 0;
  for (int i = 0; i < 3; i++) {
    float d = std::abs(glm::dot(axes_ref[i], effective_penetration_axis));
    if (d > max_dot) {
      max_dot = d;
      face_axis = i;
    }
  }

  // The other two axes define the side planes
  int side1 = (face_axis + 1) % 3;
  int side2 = (face_axis + 2) % 3;

  // Clip against 4 side planes
  glm::vec3 p1 = v1, p2 = v2;
  ClampSegmentToPlane(
      p1, p2, box_ref.position + box_ref.HalfExtents()[side1] * axes_ref[side1],
      -axes_ref[side1]);
  ClampSegmentToPlane(
      p1, p2, box_ref.position - box_ref.HalfExtents()[side1] * axes_ref[side1],
      axes_ref[side1]);
  ClampSegmentToPlane(
      p1, p2, box_ref.position + box_ref.HalfExtents()[side2] * axes_ref[side2],
      -axes_ref[side2]);
  ClampSegmentToPlane(
      p1, p2, box_ref.position - box_ref.HalfExtents()[side2] * axes_ref[side2],
      axes_ref[side2]);
  return {p1, p2};
}

std::vector<glm::vec3> Collisions::ClipCornerToFace(
    const Box& box_inc, const Box& box_ref, const glm::vec3 penetration_axis,
    const std::array<glm::vec3, 3>& axes_inc,
    const std::array<glm::vec3, 3>& axes_ref) {
  glm::vec3 corner = box_inc.position;
  for (int i = 0; i < 3; i++) {
    float sign = glm::dot(axes_inc[i], -penetration_axis) > 0 ? 1.0f : -1.0f;
    corner += sign * box_inc.HalfExtents()[i] * axes_inc[i];
  }
  return {corner};
};

std::vector<glm::vec3> Collisions::ClipEdgeEdge(
    const Box& box_a, const Box& box_b, glm::vec3 penetration_axis,
    const std::array<glm::vec3, 3>& axes_a,
    const std::array<glm::vec3, 3>& axes_b) {
  return {};
}

std::vector<glm::vec3> Collisions::ClipPolygonAgainstPlane(
    const std::vector<glm::vec3>& polygon, glm::vec3 plane_point,
    glm::vec3 plane_normal, float epsilon) {
  std::vector<glm::vec3> output;
  for (size_t i = 0; i < polygon.size(); i++) {
    glm::vec3 v1 = polygon[i];
    glm::vec3 v2 = polygon[(i + 1) % polygon.size()];

    float d1 = glm::dot(v1 - plane_point, plane_normal);
    float d2 = glm::dot(v2 - plane_point, plane_normal);

    if (d1 >= -epsilon) output.push_back(v1);  // v1 inside

    if ((d1 < -epsilon && d2 > epsilon) ||
        (d1 > epsilon && d2 < -epsilon)) {  // Edge crosses plane
      float interpolation = d1 / (d1 - d2);
      glm::vec3 interpolated_vertex = v1 + interpolation * (v2 - v1);
      output.push_back(interpolated_vertex);
    }
  }
  return output;
}

}  // namespace engine::physics
