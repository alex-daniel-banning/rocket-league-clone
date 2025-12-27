#include <engine/physics/Plane.hpp>

namespace engine::physics
{
const glm::vec3 Plane::DEFAULT_NORMAL = glm::vec3(0.0f, 1.0f, 0.0f);

Plane::Plane(float xsize, float zsize, glm::vec3 color)
    : size(xsize, zsize), color(color), position(0.0f), rotation(glm::quat(glm::vec3(0.0f))),
      initializedFlag(false)
{
    calculateCornerPositions();
}

Plane::Plane(float xsize, float zsize, glm::vec3 color, glm::vec3 position, glm::quat rotation)
    : size{xsize, zsize}, color(color), position(position), rotation(rotation),
      initializedFlag(false)
{
    calculateCornerPositions();
}

void Plane::calculateCornerPositions()
{
    // clang-format off
    const float hx = size.x * 0.5f;
    const float hz = size.y * 0.5f;

    cornerPositions[0] = position + (rotation * glm::vec3(-hx, 0.0f, -hz));
    cornerPositions[1] = position + (rotation * glm::vec3( hx, 0.0f, -hz));
    cornerPositions[2] = position + (rotation * glm::vec3( hx, 0.0f,  hz));
    cornerPositions[3] = position + (rotation * glm::vec3(-hx, 0.0f,  hz));
    // clang-format on
}

void Plane::initializeRenderData()
{
    createMesh(size.x, size.y, color);
    this->initializedFlag = true;
}

const render::Mesh *Plane::getMesh() const
{
    assert(initializedFlag && "mesh has not been initialized");
    return mesh;
}

glm::vec3 Plane::getNormal() const { return rotation * DEFAULT_NORMAL; }

glm::vec3 Plane::getMin() const
{
    // find all vertex values after rotation is applied
    // todo, confusing that size.y correlates to z value (because plane defaults in xz plane)
    // todo, if I'm going to store the points on the instance, it probably is worth decoupling the
    // createMesh function
    std::vector<glm::vec3> points = {
        rotation * glm::vec3(size.x / 2, 0.0f, size.y / 2),
        rotation * glm::vec3(size.x / 2, 0.0f, -size.y / 2),
        rotation * glm::vec3(-size.x / 2, 0.0f, size.y / 2),
        rotation * glm::vec3(-size.x / 2, 0.0f, -size.y / 2),
    };

    glm::vec3 minPos = points[0];
    for (const glm::vec3 p : points)
    {
        minPos.x = glm::min(minPos.x, p.x);
        minPos.y = glm::min(minPos.y, p.y);
        minPos.z = glm::min(minPos.z, p.z);

        // This should behave the same way (component-wise, that is)
        // minPos = glm::min(minPos, p);
    }

    return minPos;
}

glm::vec3 Plane::getMax() const { return glm::vec3(0.0f); }

glm::vec3 Plane::getPosition() const { return this->position; }

void Plane::setPosition(const glm::vec3 p)
{
    this->position = p;
    calculateCornerPositions();
}

glm::quat Plane::getRotation() const { return this->rotation; }

void Plane::setRotation(const glm::quat r)
{
    this->rotation = r;
    calculateCornerPositions();
}

engine::render::Mesh Plane::createMesh(float xsize, float zsize, glm::vec3 color)
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
    return render::Mesh(vertices, indices, color);
}
} // namespace engine::physics
