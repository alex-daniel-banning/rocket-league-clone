#pragma once

#include <engine/render/shader.hpp>
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace engine::render {

struct Vertex {
  glm::vec3 position;
  glm::vec3 normal;
  glm::vec2 tex_coords;
};

struct Texture {
  unsigned int id;
  std::string type;
  std::string path;
};

class Mesh {
 public:
  // mesh data
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  std::vector<Texture> textures;

  // used if there is no texture?
  glm::vec3 color;

  Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices,
       const std::vector<Texture>& textures);
  Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, const glm::vec3& color);
  void Draw(engine::render::Shader& shader) const;

 private:
  // render data
  unsigned int vao_, vbo_, ebo_;

  void SetupMesh();
};
}  // namespace engine::render
