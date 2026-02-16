#include <glad/glad.h>

#include <cstddef>
#include <engine/render/mesh.hpp>

namespace engine::render {

Mesh::Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices,
           const std::vector<Texture>& textures) {
  this->vertices = vertices;
  this->indices = indices;
  this->textures = textures;

  SetupMesh();
}

Mesh::Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, const glm::vec3& color) {
  this->vertices = vertices;
  this->indices = indices;
  this->color = color;

  SetupMesh();
}

void Mesh::SetupMesh() {
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);

  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);

  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

  // vertex positions
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), static_cast<const void*>(nullptr));
  // vertex normals
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<const void*>(offsetof(Vertex, normal)));
  // vertex normals
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<const void*>(offsetof(Vertex, tex_coords)));

  glBindVertexArray(0);
};

void Mesh::Draw(engine::render::Shader& shader) const {
  unsigned int diffuse_nr = 1;
  unsigned int specular_nr = 1;
  for (unsigned int i = 0; i < textures.size(); i++) {
    glActiveTexture(GL_TEXTURE0 + i);
    std::string number;
    std::string name = textures[i].type;
    if (name == "texture_diffuse") {
      number = std::to_string(diffuse_nr++);
    } else if (name == "texture_specular") {
      number = std::to_string(specular_nr++);
    }

    shader.SetInt(("material." + name + number).c_str(), i);
    glBindTexture(GL_TEXTURE_2D, textures[i].id);
  }
  if (textures.size() == 0) {
    shader.SetVec3("diffuseColor", color.r, color.g, color.b);
  }
  glActiveTexture(GL_TEXTURE0);

  // draw mesh
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}
}  // namespace engine::render
