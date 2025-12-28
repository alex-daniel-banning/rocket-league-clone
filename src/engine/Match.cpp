#include <engine/Match.hpp>

namespace engine
{

void Match::tick(float deltaTime)
{
    ball.position += deltaTime * ball.velocity;
    handleCollisions();
}

void Match::handleCollisions()
{
    // determine if the ball collides with any of the walls
    for (physics::Plane wall : walls)
    {
        // todo, what if it collides with two walls at once (corner)?
        if (ballCollides(wall))
        {
            // handle
            // The velocity component that is parallel with the wall's normal should be inversed
            // The component that is parallel to the wall should be unaffected (for elastic
            // collision)
            glm::vec3 vPerpendicular = glm::dot(ball.velocity, wall.getNormal()) * wall.getNormal();
            glm::vec3 vParallel      = ball.velocity - vPerpendicular;
            ball.velocity            = vParallel - vPerpendicular;
        }
    }
    // determine if the ball collides with the ground
    if (ballCollides(ground))
    {
        glm::vec3 vPerpendicular = glm::dot(ball.velocity, ground.getNormal()) * ground.getNormal();
        glm::vec3 vParallel      = ball.velocity - vPerpendicular;
        ball.velocity            = vParallel - vPerpendicular;
    }
}

bool Match::ballCollides(const physics::Plane &plane)
{
    // find closes point on the aabb to the sphere center
    glm::vec3 closest;
    closest.x = glm::clamp(ball.position.x, plane.getMin().x, plane.getMax().x);
    closest.y = glm::clamp(ball.position.y, plane.getMin().y, plane.getMax().y);
    closest.z = glm::clamp(ball.position.z, plane.getMin().z, plane.getMax().z);

    float distanceSquared = glm::dot(closest - ball.position, closest - ball.position);
    return distanceSquared <= ball.radius * ball.radius;
}
} // namespace engine
