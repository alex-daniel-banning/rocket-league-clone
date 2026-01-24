#include <engine/physics/Plane.hpp>
#include <iostream>

namespace engine::physics
{
const glm::vec3 Plane::DEFAULT_NORMAL = glm::vec3(0.0f, 1.0f, 0.0f);

Plane::Plane(float xLength, float zLength, glm::vec3 color, glm::vec3 position, glm::quat rotation)
    : xLength(xLength), zLength(zLength), color(color), position(position), rotation(rotation)
{
    calculateCornerPositions();
}

void Plane::calculateCornerPositions()
{
    // clang-format off
    const float hx = xLength * 0.5f;
    const float hz = zLength * 0.5f;

    cornerPositions[0] = position + (rotation * glm::vec3(-hx, 0.0f, -hz));
    cornerPositions[1] = position + (rotation * glm::vec3( hx, 0.0f, -hz));
    cornerPositions[2] = position + (rotation * glm::vec3( hx, 0.0f,  hz));
    cornerPositions[3] = position + (rotation * glm::vec3(-hx, 0.0f,  hz));
    // clang-format on
}

glm::vec3 Plane::getMin() const
{
    glm::vec3 minPos = cornerPositions[0];
    for (const glm::vec3 p : cornerPositions)
    {
        minPos.x = glm::min(minPos.x, p.x);
        minPos.y = glm::min(minPos.y, p.y);
        minPos.z = glm::min(minPos.z, p.z);

        // This should behave the same way (component-wise, that is)
        // minPos = glm::min(minPos, p);
    }

    return minPos;
}

glm::vec3 Plane::getMax() const
{
    glm::vec3 maxPos = cornerPositions[0];
    for (const glm::vec3 p : cornerPositions)
    {
        maxPos.x = glm::max(maxPos.x, p.x);
        maxPos.y = glm::max(maxPos.y, p.y);
        maxPos.z = glm::max(maxPos.z, p.z);

        // This should behave the same way (component-wise, that is)
        // maxPos = glm::max(maxPos, p);
    }

    return maxPos;
}

void Plane::setPosition(const glm::vec3 p)
{
    position = p;
    calculateCornerPositions();
}

void Plane::setRotation(const glm::quat r)
{
    rotation = r;
    calculateCornerPositions();
}

} // namespace engine::physics
