#include <engine/physics/Plane.hpp>
#include <iostream>

namespace engine::physics
{
const glm::vec3 Plane::DEFAULT_NORMAL = glm::vec3(0.0f, 1.0f, 0.0f);

Plane::Plane(float xsize, float zsize)
    : size(xsize, zsize), position(0.0f), mesh(createMesh(xsize, zsize))
{
    rotation = glm::quat(glm::vec3(0.0f));
}

glm::vec3 Plane::getNormal() const { return rotation * DEFAULT_NORMAL; }

engine::render::Mesh Plane::createMesh(float xsize, float zsize)
{
    std::vector<render::Vertex> vertices;
    // clang-format off
        float x = xsize / 2.0f;
        float z = zsize / 2.0f;
        std::vector<glm::vec3> positions = {
            glm::vec3(-x, 0.0f, z),
            glm::vec3( x, 0.0f, z),
            glm::vec3( x, 0.0f,-z),
            glm::vec3(-x, 0.0f,-z)
        };
        std::vector<glm::vec2> texCoords = {
            glm::vec2(0.0f, 1.0f),
            glm::vec2(1.0f, 1.0f),
            glm::vec2(1.0f, 0.0f),
            glm::vec2(0.0f, 0.0f)
        };
        for (int i = 0; i < 4; i++)
        {
            render::Vertex v;
            v.Position  = positions[i];
            v.Normal    = DEFAULT_NORMAL;
            v.TexCoords = texCoords[i];
            vertices.push_back(v);
        }
        std::vector<unsigned int> indices {
            0, 1, 2,
            2, 3, 0
        };
    // clang-format on
    glm::vec3 color(0.5f);
    return render::Mesh(vertices, indices, color);
}
} // namespace engine::physics
