#include <engine/physics/Plane.hpp>

namespace engine::physics
{
const glm::vec3 Plane::DEFAULT_NORMAL = glm::vec3(0.0f, 1.0f, 0.0f);

Plane::Plane(float xsize, float zsize, glm::vec3 color)
    : size(xsize, zsize), color(color), position(0.0f), rotation(glm::quat(glm::vec3(0.0f)))
{
    calculateCornerPositions();
}

Plane::Plane(float xsize, float zsize, glm::vec3 color, glm::vec3 position, glm::quat rotation)
    : size{xsize, zsize}, color(color), position(position), rotation(rotation)
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

} // namespace engine::physics
