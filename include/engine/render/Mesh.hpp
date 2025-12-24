#pragma once

#include <engine/render/Shader.hpp>
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace engine::render
{

struct Vertex
{
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
};

struct Texture
{
    unsigned int id;
    std::string type;
    std::string path;
};

class Mesh
{
  public:
    // mesh data
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;

    // used if there is no texture?
    glm::vec3 color;

    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices,
         std::vector<Texture> textures);
    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices, glm::vec3 color);
    void Draw(engine::render::Shader &shader) const;

  private:
    // render data
    unsigned int VAO, VBO, EBO;

    void setupMesh();
};
} // namespace engine::render
