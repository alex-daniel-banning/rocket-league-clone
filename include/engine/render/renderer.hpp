#pragma once

#include <engine/physics/box.hpp>
#include <engine/physics/plane.hpp>
#include <engine/physics/sphere.hpp>
#include <engine/render/camera.hpp>
#include <engine/render/model.hpp>
#include <engine/render/shader.hpp>
#include <glm/gtc/quaternion.hpp>

namespace engine::render {

class Renderer {
 public:
  Renderer(float screen_width, float screen_height);
  void DrawBox(const engine::physics::Box& box, Shader& shader, const Camera& camera);
  void DrawBox(const engine::physics::Box& box, Shader& shader);
  void DrawBoxWireframe(const engine::physics::Box& box, Shader& shader, const Camera& camera);
  void DrawSphere(const engine::physics::Sphere& sphere, Shader& shader, const Camera& camera);
  void DrawSphere(const engine::physics::Sphere& sphere, Shader& shader);
  void DrawSphereWireframe(const engine::physics::Sphere& sphere, Shader& shader, const Camera& camera);

  // Draws a screen-space orientation gizmo (world axes: +X red, +Y green, +Z
  // blue) in the top-right corner, reflecting the camera's current rotation.
  void DrawOrientationGizmo(const Camera& camera, Shader& shader);

  void DrawModel(const Model& model, Shader& shader, const Camera& camera, const glm::vec3& position,
                 const glm::vec3& scale, const glm::quat& rotation);
  void DrawPhysicsPlane(const physics::Plane& plane, Shader& shader);
  void DrawPhysicsPlane(const physics::Plane& plane, Shader& shader, const Camera& camera);
  void DrawPhysicsPlaneNormal(const physics::Plane& plane, Shader& shader, const Camera& camera);

  glm::mat4 MakeModelMatrix(const engine::physics::Box& box);
  glm::mat4 MakeModelMatrix(const engine::physics::Sphere& sphere);

  // todo, I don't know what these are useful for, but chat gpt suggested
  // void beginFrame();
  // void endFrame();

 private:
  float screen_width_, screen_height_;
  float near_plane_, far_plane_;
  unsigned int cube_vao_, cube_vbo_;
  unsigned int cube_wire_vao_, cube_wire_vbo_, cube_wire_ebo_;
  unsigned int sphere_vao_, sphere_ebo_, sphere_index_count_;
  unsigned int plane_vao_, plane_vbo_;
  unsigned int line_vao_, line_vbo_;
  void InitCube();
  void InitWireCube();
  void InitSphere();
  void InitPlane();
  void InitLine();
};

}  // namespace engine::render
