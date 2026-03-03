#include "engine/physics/collisions.hpp"

#include <stdexcept>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include "engine/log.hpp"

namespace {
glm::vec3 CalculateCentroid(const std::vector<glm::vec3>& points) {
  assert(points.size() > 0);
  if (points.size() == 1) return points[0];
  glm::vec3 sum(0.0f);
  for (const auto& p : points) sum += p;
  return sum / static_cast<float>(points.size());
}

std::array<glm::vec3, 3> GetAxesFromQuaternion(const glm::quat& q) {  // clang-format off
  glm::mat3 mat3 = glm::toMat3(q);
  return {glm::normalize(mat3[0]), glm::normalize(mat3[1]), glm::normalize(mat3[2])};
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
    const auto& v1 = polygon[i];
    const auto& v2 = polygon[(i + 1) % polygon.size()];

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
  // If the dot product of the ref_axis and penetration is negative, then the
  // ref_axis is pointing in the correct direction and doesn't need to be
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

std::vector<glm::vec3> ClipCornerToFace(const engine::physics::Box& box_ref, const engine::physics::Box& box_inc,
                                        const glm::vec3& penetration_axis, const std::array<glm::vec3, 3>& axes_ref,
                                        const std::array<glm::vec3, 3>& axes_inc) {
  glm::vec3 corner = box_inc.position;
  for (int i = 0; i < 3; i++) {
    float sign = glm::dot(axes_inc[i], penetration_axis) < 0 ? 1.0f : -1.0f;
    corner += sign * box_inc.HalfExtents()[i] * axes_inc[i];
  }
  return {corner};
};

void CorrectPenetration(glm::vec3& position_a, float inv_mass_a, glm::vec3& position_b, float inv_mass_b,
                        const glm::vec3& normal, float penetration) {
  if (penetration <= 0.0f) return;
  const float separation_slop = 1e-3f;
  float total_correction = penetration + separation_slop;
  float total_inv_mass = inv_mass_a + inv_mass_b;
  position_a -= normal * (total_correction / total_inv_mass) * inv_mass_a;
  position_b += normal * (total_correction / total_inv_mass) * inv_mass_b;
}

glm::mat3 WorldInverseInertia(const glm::quat& rotation, const glm::mat3& inertia_tensor_inv) {
  glm::mat3 rot = glm::toMat3(rotation);
  return rot * inertia_tensor_inv * glm::transpose(rot);
}

float AngularMassContribution(const glm::mat3& i_world_inv, const glm::vec3& r, const glm::vec3& n) {
  return glm::dot(n, glm::cross(i_world_inv * glm::cross(r, n), r));
}

void ApplyImpulse(glm::vec3& velocity, float mass_inv, glm::vec3& angular_velocity, const glm::mat3& i_world_inv,
                  const glm::vec3& r, const glm::vec3& impulse) {
  velocity += impulse * mass_inv;
  angular_velocity += i_world_inv * glm::cross(r, impulse);
}

void ResolveBoxSphereCollision(engine::physics::Box& box, engine::physics::Sphere& sphere,
                               const engine::physics::Contact& contact, float coefficient_of_restitution) {
  // This is all from the perspective of the sphere
  glm::vec3 omega = box.angular_velocity;
  glm::vec3 r = contact.points[0] - box.position;
  glm::vec3 v_rel = sphere.velocity - (box.velocity + glm::cross(omega, r));
  glm::vec3 n = contact.normal;
  glm::mat3 i_world_inv = WorldInverseInertia(box.rotation, box.inertia_tensor_inv);
  float rel_vel_along_normal = glm::dot(v_rel, n);

  // Objects are geometrically penetrating but already separating — this is
  // valid in multi-body simulation (resolving one contact can leave another
  // pair overlapping but diverging). Applying an impulse here would pull them
  // together, so skip.
  if (rel_vel_along_normal > 0.0f) return;

  float effective_mass = sphere.mass_inv + box.mass_inv + AngularMassContribution(i_world_inv, r, n);

  float impulse_scalar = -(1 + coefficient_of_restitution) * rel_vel_along_normal / effective_mass;
  glm::vec3 impulse_vector = impulse_scalar * n;

  LOG_DEBUG("box-sphere resolve: j=%.4f v_n=%.4f m_eff=%.4f pen=%.4f", impulse_scalar, rel_vel_along_normal,
            effective_mass, contact.penetration);

  sphere.velocity += impulse_vector * sphere.mass_inv;
  ApplyImpulse(box.velocity, box.mass_inv, box.angular_velocity, i_world_inv, r, -impulse_vector);

  CorrectPenetration(box.position, box.mass_inv, sphere.position, sphere.mass_inv, contact.normal, contact.penetration);
}

void ResolveBoxBoxCollision(engine::physics::Box& box_a, engine::physics::Box& box_b,
                            const engine::physics::Contact& contact, float coefficient_of_restitution) {
  if (box_a.mass_inv == 0.0f && box_b.mass_inv == 0.0f) {
    throw std::logic_error("Two immovable objects should not have collision resolution applied.");
  }

  glm::vec3 impulse_centroid = CalculateCentroid(contact.points);
  glm::vec3 r_a = impulse_centroid - box_a.position;
  glm::vec3 r_b = impulse_centroid - box_b.position;
  glm::vec3 v_contact_a = box_a.velocity + glm::cross(box_a.angular_velocity, r_a);
  glm::vec3 v_contact_b = box_b.velocity + glm::cross(box_b.angular_velocity, r_b);
  glm::vec3 rel_vel = v_contact_a - v_contact_b;
  glm::mat3 i_world_inv_a = WorldInverseInertia(box_a.rotation, box_a.inertia_tensor_inv);
  glm::mat3 i_world_inv_b = WorldInverseInertia(box_b.rotation, box_b.inertia_tensor_inv);
  float m_eff = box_a.mass_inv + box_b.mass_inv + AngularMassContribution(i_world_inv_a, r_a, contact.normal) +
                AngularMassContribution(i_world_inv_b, r_b, contact.normal);
  float v_n = glm::dot(rel_vel, contact.normal);
  if (v_n < 0.0f) return;  // already separating, skip

  float j = -(1.0f + coefficient_of_restitution) * v_n / m_eff;
  glm::vec3 impulse = j * contact.normal;

  LOG_DEBUG("box-box resolve: j=%.4f v_n=%.4f m_eff=%.4f contacts=%zu pen=%.4f", j, v_n, m_eff, contact.points.size(),
            contact.penetration);

  ApplyImpulse(box_a.velocity, box_a.mass_inv, box_a.angular_velocity, i_world_inv_a, r_a, impulse);
  ApplyImpulse(box_b.velocity, box_b.mass_inv, box_b.angular_velocity, i_world_inv_b, r_b, -impulse);

  CorrectPenetration(box_a.position, box_a.mass_inv, box_b.position, box_b.mass_inv, contact.normal,
                     contact.penetration);
}

};  // namespace

namespace engine::physics {

bool Collisions::ComputeContact(const Box& box, const Sphere& sphere, Contact& out) {
  glm::vec3 sphere_to_box = sphere.position - box.position;

  glm::vec3 closest_point = box.position;
  std::array<glm::vec3, 3> obb_axes = {box.rotation * glm::vec3(1.0f, 0.0f, 0.0f),
                                       box.rotation * glm::vec3(0.0f, 1.0f, 0.0f),
                                       box.rotation * glm::vec3(0.0f, 0.0f, 1.0f)};
  for (unsigned int i = 0; i < 3; i++) {
    float distance = glm::dot(sphere_to_box, obb_axes[i]);
    distance = glm::clamp(distance, -box.HalfExtents()[i], box.HalfExtents()[i]);
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

void Collisions::HandleCollision(Box& box, Sphere& sphere, float restitution) {
  Contact contact;
  if (!ComputeContact(box, sphere, contact)) return;
  ResolveBoxSphereCollision(box, sphere, contact, restitution);
}

bool Collisions::ComputeContact(const Box& box_a, const Box& box_b, Contact& out) {
  const float epsilon = 1e-6f;
  float penetration = std::numeric_limits<float>::max();
  glm::vec3 penetration_axis;

  const std::array<glm::vec3, 3> axes_a = GetAxesFromQuaternion(box_a.rotation);
  const std::array<glm::vec3, 3> axes_b = GetAxesFromQuaternion(box_b.rotation);

  enum class AxisSource { FACE_A, FACE_B, EDGE_EDGE };
  AxisSource axis_source;

  size_t axis_count = 0;
  std::array<glm::vec3, 15> axes_to_test;
  for (const auto& axis : axes_a) axes_to_test[axis_count++] = axis;
  for (const auto& axis : axes_b) axes_to_test[axis_count++] = axis;
  for (const auto& edge_a : axes_a) {
    for (const auto& edge_b : axes_b) {
      glm::vec3 axis = glm::cross(edge_a, edge_b);
      if (glm::length(axis) >= epsilon) axes_to_test[axis_count++] = glm::normalize(axis);
    }
  }

  for (size_t i = 0; i < axis_count; i++) {
    float overlap = CalculateOverlap(axes_to_test[i], box_a, box_b, axes_a, axes_b);
    if (overlap <= 0) return false;
    // Bias edge-edge overlaps so face axes are preferred when close
    if (overlap < penetration) {
      penetration = overlap;
      penetration_axis = axes_to_test[i];
      // Penetration axis must always point from a to b. Consequence: v_n > 0
      // means approaching, v_n < 0 means separating (opposite of sphere case).
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
  bool a_clips_b = true;
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
      if (axis_source == AxisSource::FACE_A) {
        contact_points = ClipFaceFace(box_a, box_b, penetration_axis, axes_a, axes_b);
      } else {
        contact_points = ClipFaceFace(box_b, box_a, -penetration_axis, axes_b, axes_a);
        a_clips_b = false;
      }
    } else {
      if (axis_source == AxisSource::FACE_A) {
        contact_points = ClipCornerToFace(box_a, box_b, penetration_axis, axes_a, axes_b);
      } else {
        contact_points = ClipCornerToFace(box_b, box_a, -penetration_axis, axes_b, axes_a);
        a_clips_b = false;
      }
    }
  }

  // Clip in the correct direction.
  const float sign = a_clips_b ? 1.0f : -1.0f;
  for (auto& p : contact_points) {
    p = p + sign * (0.5f * penetration * penetration_axis);
  }
  // Clipping can return empty results in degenerate cases (parallel edges in
  // ClipEdgeEdge, or floating-point drift pushing all vertices to the wrong
  // side of the reference face in ClipFaceFace). Skip resolution this substep.
  if (contact_points.empty()) return false;
  out.points.insert(out.points.end(), contact_points.begin(), contact_points.end());
  out.normal = penetration_axis;
  out.penetration = penetration;
  return true;
}

void Collisions::HandleCollision(Box& box_a, Box& box_b, float restitution) {
  Contact contact;
  if (!ComputeContact(box_a, box_b, contact)) return;
  ResolveBoxBoxCollision(box_a, box_b, contact, restitution);
}

}  // namespace engine::physics
