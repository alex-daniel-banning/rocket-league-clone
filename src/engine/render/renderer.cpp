#include <glad/glad.h>

#include <engine/render/camera.hpp>
#include <engine/render/renderer.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace engine::render {
// clang-format off
static constexpr float cube_vertices[] = 
{
    // Positions.......   Normals...........
  -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
   0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
   0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
   0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
  -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
  -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
  -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
   0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
   0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
   0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
  -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
  -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
  -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
  -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
  -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
  -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
  -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
  -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
   0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
   0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
   0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
   0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
   0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
   0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
  -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
  -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
   0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
   0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
   0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
  -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
  -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
   0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
  -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
   0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
  -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
   0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f
};

static constexpr float plane_vertices[] = {
  // Positions.......   Normals...........
  -0.5f,  0.0f, -0.5f,  0.0f,  1.0f,  0.0f,
   0.5f,  0.0f, -0.5f,  0.0f,  1.0f,  0.0f,
   0.5f,  0.0f,  0.5f,  0.0f,  1.0f,  0.0f,
   0.5f,  0.0f,  0.5f,  0.0f,  1.0f,  0.0f,
  -0.5f,  0.0f,  0.5f,  0.0f,  1.0f,  0.0f,
  -0.5f,  0.0f, -0.5f,  0.0f,  1.0f,  0.0f,
};

static const float cube_wire_vertices[8][3] = {
  {-0.5f, -0.5f, -0.5f},
  { 0.5f, -0.5f, -0.5f},
  { 0.5f,  0.5f, -0.5f},
  {-0.5f,  0.5f, -0.5f},
  {-0.5f, -0.5f,  0.5f},
  { 0.5f, -0.5f,  0.5f},
  { 0.5f,  0.5f,  0.5f},
  {-0.5f,  0.5f,  0.5f}
};

static constexpr unsigned int cube_wire_indices[] = {
  0, 1, 1,
  2, 2, 3,
  3, 0, 4,
  5, 5, 6,
  6, 7, 7,
  4, 0, 4,
  1, 5, 2,
  6, 3, 7
};
// clang-format on
Renderer::Renderer(const float screen_width, const float screen_height) {
  this->screen_width_ = screen_width;
  this->screen_height_ = screen_height;
  InitCube();
  InitWireCube();
  InitSphere();
  InitPlane();
  InitLine();
}

void Renderer::DrawBox(const engine::physics::Box& box,
                       engine::render::Shader& shader, const Camera& camera) {
  glm::mat4 model = glm::mat4(1.0f);
  glm::mat4 view;
  view = camera.GetViewMatrix();
  glm::mat4 projection;
  projection = glm::perspective(
      camera.projection.fov, screen_width_ / screen_height_,
      camera.projection.near_plane, camera.projection.far_plane);
  model = glm::translate(model, box.position);
  model = model * glm::toMat4(box.rotation);
  model =
      glm::scale(model, glm::vec3(box.Size().x, box.Size().y, box.Size().z));
  shader.SetMat4("model", model);
  shader.SetMat4("view", view);
  shader.SetMat4("projection", projection);
  shader.SetVec3("diffuseColor", glm::vec3(0.4f, 0.4f, 0.65f));
  glBindVertexArray(cube_vao_);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0);
}

void Renderer::DrawBox(const engine::physics::Box& box, Shader& shader) {
  glm::mat4 model_matrix = glm::mat4(1.0f);
  model_matrix = glm::translate(model_matrix, box.position);
  model_matrix = model_matrix * glm::toMat4(box.rotation);
  model_matrix = glm::scale(model_matrix, glm::vec3(box.Size()));
  shader.SetMat4("model", model_matrix);
  glBindVertexArray(cube_vao_);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0);
}

void Renderer::DrawBoxWireframe(const engine::physics::Box& box,
                                engine::render::Shader& shader,
                                const Camera& camera) {
  glm::mat4 model = glm::mat4(1.0f);
  glm::mat4 view;
  view = camera.GetViewMatrix();
  glm::mat4 projection;
  projection = glm::perspective(
      camera.projection.fov, screen_width_ / screen_height_,
      camera.projection.near_plane, camera.projection.far_plane);
  model = glm::translate(model, box.position);
  model = model * glm::toMat4(box.rotation);
  model =
      glm::scale(model, glm::vec3(box.Size().x, box.Size().y, box.Size().z));
  shader.Use();
  shader.SetMat4("model", model);
  shader.SetMat4("view", view);
  shader.SetMat4("projection", projection);
  shader.SetVec3("diffuseColor", glm::vec3(0.0f, 1.0f, 0.0f));

  glLineWidth(2.0f);
  glEnable(GL_LINE_SMOOTH);
  glBindVertexArray(cube_wire_vao_);
  glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void Renderer::DrawSphere(const engine::physics::Sphere& sphere,
                          engine::render::Shader& shader,
                          const Camera& camera) {
  glm::mat4 model = MakeModelMatrix(sphere);
  glm::mat4 view = camera.GetViewMatrix();
  glm::mat4 projection = glm::perspective(
      camera.projection.fov, screen_width_ / screen_height_,
      camera.projection.near_plane, camera.projection.far_plane);

  shader.Use();
  shader.SetMat4("model", model);
  shader.SetMat4("view", view);
  // shader.setMat4("projection", projection);
  glBindVertexArray(sphere_vao_);
  glDrawElements(GL_TRIANGLES, sphere_index_count_, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void Renderer::DrawSphere(const engine::physics::Sphere& sphere,
                          Shader& shader) {
  glm::mat4 model_matrix = glm::mat4(1.0f);
  model_matrix = glm::translate(model_matrix, sphere.position);
  model_matrix = glm::scale(model_matrix, glm::vec3(sphere.radius));
  shader.SetMat4("model", model_matrix);
  glBindVertexArray(sphere_vao_);
  glDrawElements(GL_TRIANGLES, sphere_index_count_, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void Renderer::DrawSphereWireframe(const engine::physics::Sphere& sphere,
                                   engine::render::Shader& shader,
                                   const Camera& camera) {
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  DrawSphere(sphere, shader, camera);
}

// todo, remove this, too coupled perhaps
void Renderer::DrawModel(engine::render::Model& model,
                         engine::render::Shader& shader, const Camera& camera,
                         glm::vec3 position, glm::vec3 scale,
                         glm::quat rotation) {
  shader.Use();
  glm::mat4 model_matrix = glm::mat4(1.0f);
  model_matrix = glm::translate(model_matrix, position);
  model_matrix = model_matrix * glm::mat4_cast(rotation);
  model_matrix = glm::scale(model_matrix, scale);
  glm::mat4 view;
  view = camera.GetViewMatrix();
  glm::mat4 projection;
  projection = glm::perspective(
      camera.projection.fov, screen_width_ / screen_height_,
      camera.projection.near_plane, camera.projection.far_plane);
  shader.SetMat4("model", model_matrix);
  shader.SetMat4("view", view);
  shader.SetMat4("projection", projection);
  shader.SetVec3("lightColor", 1.0f, 1.0f, 1.0f);
  shader.SetVec3("lightPos", 5.0f, 5.0f, 5.0f);
  model.Draw(shader);
}

// this hasn't been tested that it works. Current known use case is for
// generating shadow map, which is why we don't want the camera's projection.
// Haven't created a scene yet that has a plane cast a shadow.
void Renderer::DrawPhysicsPlane(const physics::Plane& plane, Shader& shader) {
  glm::mat4 model_matrix = glm::mat4(1.0f);
  model_matrix = glm::translate(model_matrix, plane.GetPosition());
  model_matrix = model_matrix * glm::mat4_cast(plane.GetRotation());
  model_matrix = glm::scale(model_matrix, glm::vec3(1.0f));
  shader.SetMat4("model", model_matrix);
  glBindVertexArray(plane_vao_);
  glDrawArrays(GL_TRIANGLES, 0, 6);
  glBindVertexArray(0);
}

void Renderer::DrawPhysicsPlane(const physics::Plane& plane, Shader& shader,
                                const Camera& camera) {
  glm::mat4 model_matrix = glm::mat4(1.0f);
  glm::mat4 view;
  view = camera.GetViewMatrix();
  glm::mat4 projection;
  projection = glm::perspective(
      camera.projection.fov, screen_width_ / screen_height_,
      camera.projection.near_plane, camera.projection.far_plane);

  model_matrix = glm::translate(model_matrix, plane.GetPosition());
  model_matrix = model_matrix * glm::mat4_cast(plane.GetRotation());
  model_matrix = glm::scale(
      model_matrix, glm::vec3(plane.GetXLength(), 1.0f, plane.GetZLength()));
  shader.SetMat4("model", model_matrix);
  shader.SetMat4("view", view);
  shader.SetMat4("projection", projection);
  shader.SetVec3("diffuseColor", plane.color);
  glBindVertexArray(plane_vao_);
  glDrawArrays(GL_TRIANGLES, 0, 6);
  glBindVertexArray(0);
}

void Renderer::DrawPhysicsPlaneNormal(const physics::Plane& plane,
                                      Shader& shader, const Camera& camera) {
  glm::mat4 model_matrix = glm::mat4(1.0f);
  glm::mat4 view;
  view = camera.GetViewMatrix();
  glm::mat4 projection;
  projection = glm::perspective(
      camera.projection.fov, screen_width_ / screen_height_,
      camera.projection.near_plane, camera.projection.far_plane);

  model_matrix = glm::translate(model_matrix, plane.GetPosition());
  model_matrix = model_matrix * glm::mat4_cast(plane.GetRotation());
  model_matrix = glm::scale(model_matrix, glm::vec3(1.0f));
  shader.SetMat4("model", model_matrix);
  shader.SetMat4("view", view);
  shader.SetMat4("projection", projection);
  glBindVertexArray(line_vao_);
  glDrawArrays(GL_LINES, 0, 2);
  glBindVertexArray(0);
}

// TODO remove?
glm::mat4 Renderer::MakeModelMatrix(const engine::physics::Box& box) {
  return glm::scale(glm::mat4(1.0f), glm::vec3(box.Size()));
}

glm::mat4 Renderer::MakeModelMatrix(const engine::physics::Sphere& sphere) {
  glm::mat4 model = glm::mat4(1.0f);
  model = glm::translate(model, glm::vec3(sphere.position));
  model = glm::scale(model, glm::vec3(sphere.radius));
  return model;
}

void Renderer::InitCube() {
  glGenVertexArrays(1, &cube_vao_);
  glGenBuffers(1, &cube_vbo_);

  glBindVertexArray(cube_vao_);
  glBindBuffer(GL_ARRAY_BUFFER, cube_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(cube_vertices), cube_vertices,
               GL_STATIC_DRAW);

  // Positions
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        static_cast<const void*>(nullptr));
  glEnableVertexAttribArray(0);

  // Normals
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<const void*>(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

void Renderer::InitPlane() {
  glGenVertexArrays(1, &plane_vao_);
  glGenBuffers(1, &plane_vbo_);

  glBindVertexArray(plane_vao_);
  glBindBuffer(GL_ARRAY_BUFFER, plane_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(plane_vertices), plane_vertices,
               GL_STATIC_DRAW);

  // Positions
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        static_cast<const void*>(nullptr));
  glEnableVertexAttribArray(0);

  // Normals
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<const void*>(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

void Renderer::InitLine() {
  glm::vec3 start(0.0f, 0.0f, 0.0f);
  glm::vec3 end(0.0f, 1.0f, 0.0f);
  glm::vec3 line_vertices[2] = {start, end};
  glGenVertexArrays(1, &line_vao_);
  glGenBuffers(1, &line_vbo_);

  glBindVertexArray(line_vao_);
  glBindBuffer(GL_ARRAY_BUFFER, line_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(line_vertices), line_vertices,
               GL_STATIC_DRAW);

  // Positions
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                        static_cast<const void*>(nullptr));

  glBindVertexArray(0);
}

void Renderer::InitWireCube() {
  glGenVertexArrays(1, &cube_wire_vao_);
  glGenBuffers(1, &cube_wire_vbo_);
  glGenBuffers(1, &cube_wire_ebo_);

  glBindVertexArray(cube_wire_vao_);

  glBindBuffer(GL_ARRAY_BUFFER, cube_wire_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(cube_wire_vertices), cube_wire_vertices,
               GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                        static_cast<const void*>(nullptr));

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_wire_ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cube_wire_indices),
               cube_wire_indices, GL_STATIC_DRAW);

  glBindVertexArray(0);
}

void Renderer::InitSphere() {
  struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
  };

  unsigned int stacks = 16;
  unsigned int slices = 16;
  float radius = 1.0f;

  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;

  // top Vertex
  vertices.push_back({glm::vec3(0, radius, 0), glm::vec3(0, 1, 0)});
  for (int i = 1; i < stacks; ++i) {
    float v = i / static_cast<float>(stacks);
    float phi = v * glm::pi<float>();

    for (int j = 0; j <= slices; ++j) {
      float u = j / static_cast<float>(slices);
      float theta = u * glm::two_pi<float>();

      float x = cos(theta) * sin(phi) * radius;
      float y = cos(phi) * radius;
      float z = sin(theta) * sin(phi) * radius;

      glm::vec3 pos = glm::vec3(x, y, z);
      glm::vec3 norm = glm::normalize(pos);  // normal

      vertices.push_back({pos, norm});
    }
  }
  // bottom Vertex
  vertices.push_back({glm::vec3(0, -radius, 0), glm::vec3(0, -1, 0)});

  // Generate indices
  // top
  unsigned int top_index = 0;
  unsigned int ring_start = 1;
  for (int j = 0; j < slices; ++j) {
    indices.push_back(top_index);
    indices.push_back(ring_start + j);
    indices.push_back(ring_start + j + 1);
  }
  // middle
  unsigned int ring_vertex_count = slices + 1;

  for (unsigned int i = 0; i < stacks - 2; ++i) {
    unsigned int curr = 1 + i * ring_vertex_count;
    unsigned int next = curr + ring_vertex_count;

    for (unsigned int j = 0; j < slices; ++j) {
      unsigned int v0 = curr + j;
      unsigned int v1 = next + j;
      unsigned int v0n = v0 + 1;
      unsigned int v1n = v1 + 1;

      // two triangles per quad
      indices.push_back(v0);
      indices.push_back(v1);
      indices.push_back(v0n);

      indices.push_back(v1);
      indices.push_back(v1n);
      indices.push_back(v0n);
    }
  }
  // bottom
  unsigned int bottom_index = vertices.size() - 1;
  unsigned int last_ring_start = bottom_index - ring_vertex_count;
  for (int j = 0; j < slices; ++j) {
    indices.push_back(bottom_index);
    indices.push_back(last_ring_start + j + 1);
    indices.push_back(last_ring_start + j);
  }

  // Create VAO, VBO, and EBO
  unsigned int vao, vbo, ebo;
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);
  glGenBuffers(1, &ebo);

  glBindVertexArray(vao);

  // Vertex buffer
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex),
               vertices.data(), GL_STATIC_DRAW);

  // Element buffer
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
               indices.data(), GL_STATIC_DRAW);

  // Position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        static_cast<const void*>(nullptr));
  glEnableVertexAttribArray(0);

  // Normal attribute
  glVertexAttribPointer(
      1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
      reinterpret_cast<const void*>(offsetof(Vertex, normal)));
  glEnableVertexAttribArray(1);

  glBindVertexArray(0);

  // Store VAO/EBO count somewhere in Renderer for rendering
  this->sphere_vao_ = vao;
  this->sphere_ebo_ = ebo;
  this->sphere_index_count_ = static_cast<unsigned int>(indices.size());
}

}  // namespace engine::render
