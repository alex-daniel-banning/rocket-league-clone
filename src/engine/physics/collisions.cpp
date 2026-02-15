#include <Print.hpp>
#include <algorithm>
#include <engine/physics/collision_type.hpp>
#include <engine/physics/collisions.hpp>
#include <stdexcept>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

namespace {
glm::vec3 CalculateCentroid(const std::vector<glm::vec3>& points) {
  assert(points.size() > 0);
  if (points.size() == 1) return points[0];
  float x = 0.0f, y = 0.0f, z = 0.0f;
  for (auto& p : points) {
    x += p.x;
    y += p.y;
    z += p.z;
  }
  x = x / points.size();
  y = y / points.size();
  z = z / points.size();
  return glm::vec3(x, y, z);
}

std::array<glm::vec3, 3> GetAxesFromQuaternion(glm::quat q) {  // clang-format off
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

float CalculateOverlap(const glm::vec3& axis, const engine::physics::Box& box_a, const engine::physics::Box& box_b,
                       const std::array<glm::vec3, 3>& axes_a, const std::array<glm::vec3, 3>& axes_b) {
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

std::vector<glm::vec3> ClipPolygonAgainstPlane(const std::vector<glm::vec3>& polygon, const glm::vec3& plane_point,
                                               const glm::vec3& plane_normal, float epsilon = 0.001f) {
  std::vector<glm::vec3> output;
  for (size_t i = 0; i < polygon.size(); i++) {
    glm::vec3 v1 = polygon[i];
    glm::vec3 v2 = polygon[(i + 1) % polygon.size()];

    float d1 = glm::dot(v1 - plane_point, plane_normal);
    float d2 = glm::dot(v2 - plane_point, plane_normal);

    if (d1 >= -epsilon) output.push_back(v1);  // v1 inside

    if ((d1 < -epsilon && d2 > epsilon) || (d1 > epsilon && d2 < -epsilon)) {  // Edge crosses plane
      float interpolation = d1 / (d1 - d2);
      glm::vec3 interpolated_vertex = v1 + interpolation * (v2 - v1);
      output.push_back(interpolated_vertex);
    }
  }
  return output;
}

std::vector<glm::vec3> ClipFaceFace(const engine::physics::Box& box_ref, const engine::physics::Box& box_inc,
                                    const glm::vec3& penetration_axis, const std::array<glm::vec3, 3>& axes_ref,
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
  glm::vec3 inc_center = box_inc.position + (inc_sign * box_inc.HalfExtents()[inc_idx] * axes_inc[inc_idx]);

  std::vector<glm::vec3> incident_verts = {// clang-format off
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
  glm::vec3 ref_center = box_ref.position + (ref_sign * box_ref.HalfExtents()[ref_idx] * axes_ref[ref_idx]);

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
//
// Edge-edge contact resolution using SAT (Separating Axis Theorem). When the minimum penetration axis is a cross
// product of box edges, we find the closest points between the two responsible edge segments using the parametric
// line-segment closest-point method (cf. Ericson, "Real-Time Collision Detection", §5.1.9).
std::vector<glm::vec3> ClipEdgeEdge(const engine::physics::Box& box_ref, const engine::physics::Box& box_inc,
                                    const glm::vec3& penetration_axis, const std::array<glm::vec3, 3>& axes_ref,
                                    const std::array<glm::vec3, 3>& axes_inc) {
  // Step 1: Find which pair of axes produced the penetration axis via cross product
  int ref_edge_axis = -1, inc_edge_axis = -1;
  float best_alignment = 0.f;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      glm::vec3 cross = glm::cross(axes_ref[i], axes_inc[j]);
      float cross_length = glm::length(cross);
      if (cross_length < 1e-6f) continue;  // epsilon
      glm::vec3 cross_normalized = cross / cross_length;
      float alignment = std::abs(glm::dot(cross_normalized, penetration_axis));
      if (alignment > best_alignment) {
        best_alignment = alignment;
        ref_edge_axis = i;
        inc_edge_axis = j;
      }
    }
  }

  if (ref_edge_axis < 0) return {};

  // Step 2: Find the specific edge segment on each box
  //         Edge runs along the identified axis; pick signs on the other two axes
  //         to get the edge closest to the other box
  auto get_edge_segment = [](const engine::physics::Box& box, const std::array<glm::vec3, 3>& axes, int edge_axis_index,
                             glm::vec3 direction_to_other) -> std::pair<glm::vec3, glm::vec3> {
    glm::vec3 center = box.position;
    glm::vec3 half_extents = box.HalfExtents();

    glm::vec3 edge_midpoint = center;
    for (int i = 0; i < 3; i++) {
      if (i == edge_axis_index) continue;
      float sign = (glm::dot(direction_to_other, axes[i]) > 0.f) ? 1.f : -1.f;
      edge_midpoint += axes[i] * half_extents[i] * sign;
    }

    glm::vec3 edge_half_length = axes[edge_axis_index] * half_extents[edge_axis_index];
    return {edge_midpoint - edge_half_length, edge_midpoint + edge_half_length};
  };

  glm::vec3 ref_to_inc_direction = box_inc.position - box_ref.position;
  auto [ref_edge_start, ref_edge_end] = get_edge_segment(box_ref, axes_ref, ref_edge_axis, ref_to_inc_direction);
  auto [inc_edge_start, inc_edge_end] = get_edge_segment(box_inc, axes_inc, inc_edge_axis, -ref_to_inc_direction);

  // Step 3: Closest points between two line segments
  glm::vec3 d1 = ref_edge_end - ref_edge_start;   // ref_edge_dir
  glm::vec3 d2 = inc_edge_end - inc_edge_start;   // inc_edge_dir
  glm::vec3 r = ref_edge_start - inc_edge_start;  // start offset

  float a = glm::dot(d1, d1);  // a = d1·d1
  float c = glm::dot(d2, d2);  // c = d2·d2
  float b = glm::dot(d1, d2);  // b = d1·d2
  float d = glm::dot(d1, r);   // d = d1·r
  float e = glm::dot(d2, r);   // e = d2·r

  float denominator = a * c - b * b;

  float param_ref = glm::clamp((b * e - d * c) / denominator, 0.f, 1.f);
  float param_inc = glm::clamp((b * param_ref + e) / c, 0.f, 1.f);

  glm::vec3 closest_on_ref = ref_edge_start + d1 * param_ref;
  glm::vec3 closest_on_inc = inc_edge_start + d2 * param_inc;

  // Step 4: Contact point is the midpoint
  return {(closest_on_ref + closest_on_inc) * 0.5f};
}
std::vector<glm::vec3> ClipCornerToFace(const engine::physics::Box& box_inc, const engine::physics::Box& box_ref,
                                        const glm::vec3& penetration_axis, const std::array<glm::vec3, 3>& axes_inc,
                                        const std::array<glm::vec3, 3>& axes_ref) {
  glm::vec3 corner = box_inc.position;
  for (int i = 0; i < 3; i++) {
    float sign = glm::dot(axes_inc[i], -penetration_axis) > 0 ? 1.0f : -1.0f;
    corner += sign * box_inc.HalfExtents()[i] * axes_inc[i];
  }
  return {corner};
};

};  // namespace

namespace engine::physics {

bool Collisions::ComputeContact(const Box& box, const Sphere& sphere, Contact& out) {
  glm::vec3 sphere_to_box = sphere.position - box.position;

  glm::vec3 closest_point = box.position;
  std::array<glm::vec3, 3> obb_axes = {box.rotation * glm::vec3(1.0f, 0.0f, 0.0f),
                                       box.rotation * glm::vec3(0.0f, 1.0f, 0.0f),
                                       box.rotation * glm::vec3(0.0f, 0.0f, 1.0f)};
  std::array<float, 3> halves = {box.Size().x / 2, box.Size().y / 2, box.Size().z / 2};
  for (unsigned int i = 0; i < 3; i++) {
    float distance = glm::dot(sphere_to_box, obb_axes[i]);
    distance = glm::clamp(distance, -halves[i], halves[i]);
    closest_point = closest_point + (distance * obb_axes[i]);
  }
  glm::vec3 v_box_surface_to_sphere_center = sphere.position - closest_point;
  float distance_squared = glm::dot(v_box_surface_to_sphere_center, v_box_surface_to_sphere_center);
  bool collides = distance_squared < sphere.radius * sphere.radius;
  if (!collides) {
    return false;
  }
  out.normal = glm::normalize(v_box_surface_to_sphere_center);
  out.penetration = sphere.radius - std::sqrt(distance_squared);
  out.points.push_back(closest_point);
  return true;
};

void Collisions::ResolveCollision(Box& box, Sphere& sphere, Contact contact, float coefficient_of_restitution) {
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
  float mass_component = (1.0f / sphere.mass) + box.mass_inv;

  // Angular mass component
  glm::vec3 r_cross_n = glm::cross(r, n);
  glm::vec3 angular_contrib = glm::cross(i_world_inv * r_cross_n, r);
  float angular_mass_component = glm::dot(n, angular_contrib);

  // Total effective mass along normal
  float effective_mass = mass_component + angular_mass_component;

  float impulse_scalar = -(1 + coefficient_of_restitution) * rel_vel_along_normal / effective_mass;
  glm::vec3 impulse_vector = impulse_scalar * n;

  // Apply the impulse
  sphere.velocity += impulse_vector / sphere.mass;
  box.velocity -= impulse_vector * box.mass_inv;
  box.angular_velocity += i_world_inv * glm::cross(r, -impulse_vector);

  // Penetration correction
  if (contact.penetration > 0.0f) {
    // Separate the two along normal
    float total_correction = contact.penetration + separation_slop;
    float total_inv_mass = (1.0f / sphere.mass) + box.mass_inv;
    float sphere_correction = (total_correction / total_inv_mass) * (1.0f / sphere.mass);
    float box_correction = (total_correction / total_inv_mass) * box.mass_inv;
    sphere.position += contact.normal * sphere_correction;
    box.position -= contact.normal * box_correction;
  }
  return;
}

void Collisions::ResolveElasticCollision(Box& box, Sphere& sphere, Contact contact) {
  ResolveCollision(box, sphere, contact, 1.0f);
}

bool Collisions::ComputeContact(const Box& box_a, const Box& box_b, Contact& out) {
  const float epsilon = 1e-6f;
  float penetration = std::numeric_limits<float>::max();
  glm::vec3 penetration_axis;

  const std::array<glm::vec3, 3> axes_a = GetAxesFromQuaternion(box_a.rotation);
  const std::array<glm::vec3, 3> axes_b = GetAxesFromQuaternion(box_b.rotation);

  enum class AxisSource { FACE_A, FACE_B, EDGE_EDGE };
  AxisSource axis_source;

  std::vector<glm::vec3> axes_to_test;
  axes_to_test.reserve(15);
  axes_to_test.insert(axes_to_test.end(), axes_a.begin(), axes_a.end());
  axes_to_test.insert(axes_to_test.end(), axes_b.begin(), axes_b.end());
  for (const glm::vec3 edge_a : axes_a) {
    for (const glm::vec3 edge_b : axes_b) {
      glm::vec3 axis = glm::cross(edge_a, edge_b);
      if (glm::length(axis) >= epsilon) axes_to_test.push_back(glm::normalize(axis));
    }
  }

  for (size_t i = 0; i < axes_to_test.size(); i++) {
    float overlap = CalculateOverlap(axes_to_test[i], box_a, box_b, axes_a, axes_b);
    if (overlap <= 0) return false;
    // Bias edge-edge overlaps so face axes are preferred when close
    if (overlap < penetration) {
      penetration = overlap;
      penetration_axis = axes_to_test[i];
      if (glm::dot(box_b.position - box_a.position, penetration_axis) < 0) penetration_axis = -penetration_axis;
      if (i < 3)
        axis_source = AxisSource::FACE_A;
      else if (i < 6)
        axis_source = AxisSource::FACE_B;
      else
        axis_source = AxisSource::EDGE_EDGE;
    }
  }

  std::vector<glm::vec3> contact_points;
  if (axis_source == AxisSource::EDGE_EDGE) {
    contact_points = ClipEdgeEdge(box_a, box_b, penetration_axis, axes_a, axes_b);
  } else {
    const auto& incident_axes = (axis_source == AxisSource::FACE_A) ? axes_b : axes_a;
    bool has_parallel = false;
    bool has_perpendicular = false;
    for (const auto& axis : incident_axes) {
      float d = std::abs(glm::dot(axis, penetration_axis));
      if (d > 0.999f) has_parallel = true;
      if (d < 0.001f) has_perpendicular = true;
    }
    if (has_parallel) {
      contact_points = ClipFaceFace(box_a, box_b, penetration_axis, axes_a, axes_b);
    } else if (has_perpendicular) {
      contact_points = (axis_source == AxisSource::FACE_A)
                           ? ClipFaceFace(box_a, box_b, penetration_axis, axes_a, axes_b)
                           : ClipFaceFace(box_b, box_a, penetration_axis, axes_b, axes_a);
    } else {
      contact_points = (axis_source == AxisSource::FACE_A)
                           ? ClipCornerToFace(box_b, box_a, penetration_axis, axes_b, axes_a)
                           : ClipCornerToFace(box_a, box_b, penetration_axis, axes_a, axes_b);
    }
  }

  for (auto& p : contact_points) {
    p = p + (0.5f * penetration * penetration_axis);
  }
  out.points.insert(out.points.end(), contact_points.begin(), contact_points.end());
  out.normal = penetration_axis;
  out.penetration = penetration;
  assert(out.points.size() > 0);
  return true;
}

void Collisions::ResolveCollision(Box& box_a, Box& box_b, Contact contact, float coefficient_of_restitution) {
  if (box_a.mass_inv == 0.0f && box_b.mass_inv == 0.0f) {
    throw std::logic_error("Two immovable objects should not have collision resolution applied.");
  }
  // TODO, handle tunneling
  const float separation_slop = 1e-3f;

  const glm::vec3 box_a_initial_velocity = box_a.velocity;
  const glm::vec3 box_b_initial_velocity = box_b.velocity;
  const glm::vec3 box_a_initial_angular_velocity = box_a.angular_velocity;
  const glm::vec3 box_b_initial_angular_velocity = box_b.angular_velocity;
  glm::vec3 impulse_centroid = CalculateCentroid(contact.points);
  // Contact point relative to each body's center of mass
  glm::vec3 r_a = impulse_centroid - box_a.position;
  glm::vec3 r_b = impulse_centroid - box_b.position;
  // Velocity at contact point for each body
  glm::vec3 v_contact_a = box_a_initial_velocity + glm::cross(box_a_initial_angular_velocity, r_a);
  glm::vec3 v_contact_b = box_b_initial_velocity + glm::cross(box_b_initial_angular_velocity, r_b);
  glm::vec3 rel_vel = v_contact_a - v_contact_b;
  // Transform inertia tensors to world space
  glm::mat3 rot_a = glm::toMat3(box_a.rotation);
  glm::mat3 rot_b = glm::toMat3(box_b.rotation);
  glm::mat3 i_world_inv_a = rot_a * box_a.inertia_tensor_inv * glm::transpose(rot_a);
  glm::mat3 i_world_inv_b = rot_b * box_b.inertia_tensor_inv * glm::transpose(rot_b);
  // Compute effective mass along collision normal
  glm::vec3 r_cross_n_a = glm::cross(r_a, contact.normal);
  glm::vec3 r_cross_n_b = glm::cross(r_b, contact.normal);
  float m_eff = box_a.mass_inv + box_b.mass_inv +
                glm::dot(contact.normal, glm::cross(i_world_inv_a * r_cross_n_a, r_a)) +
                glm::dot(contact.normal, glm::cross(i_world_inv_b * r_cross_n_b, r_b));
  // Impulse magnitude
  float v_n = glm::dot(rel_vel, contact.normal);
  float j = -(1.0f + coefficient_of_restitution) * v_n / m_eff;
  glm::vec3 impulse = j * contact.normal;
  // Accumulate the impulse value
  box_a.velocity += impulse * box_a.mass_inv;
  box_b.velocity -= impulse * box_b.mass_inv;
  box_a.angular_velocity += i_world_inv_a * glm::cross(r_a, impulse);
  box_b.angular_velocity -= i_world_inv_b * glm::cross(r_b, impulse);

  // Penetration correction
  if (contact.penetration > 0.0f) {
    // Separate the two along normal
    float total_correction = contact.penetration + separation_slop;
    float total_inv_mass = box_a.mass_inv + box_b.mass_inv;
    float box_a_correction = (total_correction / total_inv_mass) * box_a.mass_inv;
    float box_b_correction = (total_correction / total_inv_mass) * box_b.mass_inv;
    box_a.position -= contact.normal * box_a_correction;
    box_b.position += contact.normal * box_b_correction;
  }
  return;
}

}  // namespace engine::physics
