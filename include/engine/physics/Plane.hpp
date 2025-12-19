#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include <engine/render/Mesh.hpp>
#include <engine/render/Shader.hpp>

#include <iostream>

namespace engine::physics
{

const static glm::vec3 DEFAULT_NORMAL = glm::vec3(0.0f, 0.0f, 1.0f);
class Plane
{
  public:
    glm::vec2 size;
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 normal;

    // Plane(float xsize, float ysize)
    //     : size(xsize, ysize), position(0.0f), rotation(glm::vec3(0.0f, 0.0f, 1.0f)),
    //       normal(DEFAULT_NORMAL), mesh(createMesh(xsize, ysize)) {};
    Plane(float xsize, float ysize)
        : size(xsize, ysize), position(0.0f), mesh(createMesh(xsize, ysize))
    {
        rotation = glm::quat(glm::vec3(glm::radians(0.0f), 0.0f, 0.0f));
        normal   = glm::rotate(rotation, DEFAULT_NORMAL);
    };

    void Draw(render::Shader &shader);

  private:
    engine::render::Mesh mesh;

    static engine::render::Mesh createMesh(float xsize, float ysize)
    {
        std::vector<render::Vertex> vertices;
        // clang-format off
        float x = xsize / 2.0f;
        float y = ysize / 2.0f;
        std::vector<glm::vec3> positions = {
            glm::vec3(-x, y, 0.0f),
            glm::vec3( x, y, 0.0f),
            glm::vec3( x,-y, 0.0f),
            glm::vec3(-x,-y, 0.0f)
        };
        //std::vector<glm::vec3> positions = {
        //    glm::vec3(-x, 0.0f, y),
        //    glm::vec3( x, 0.0f, y),
        //    glm::vec3( x, 0.0f,-y),
        //    glm::vec3(-x, 0.0f,-y)
        //};
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
            //v.Normal    = glm::vec3(0.0f, 1.0f, 0.0f);
            v.TexCoords = texCoords[i];
            vertices.push_back(v);
        }
        std::vector<unsigned int> indices {
            0, 1, 2,
            2, 3, 0
        };
        // clang-format on
        glm::vec3 color(0.5f);
        std::cout << "Printing plane data:\n";
        for (int i = 0; i < indices.size(); i++)
        {
            float vx = vertices[indices[i]].Position.x;
            float vy = vertices[indices[i]].Position.y;
            float vz = vertices[indices[i]].Position.z;
            float nx = vertices[indices[i]].Normal.x;
            float ny = vertices[indices[i]].Normal.y;
            float nz = vertices[indices[i]].Normal.z;
            float tx = vertices[indices[i]].TexCoords.x;
            float ty = vertices[indices[i]].TexCoords.y;
            std::cout << vx << ", " << vy << ", " << vz << ", " << nx << ", " << ny << ", " << nz
                      << ", " << tx << ", " << ty << "\n";
        }
        std::cout << "Finished printing plane data." << std::endl;
        return render::Mesh(vertices, indices, color);
    }
};
} // namespace engine::physics
