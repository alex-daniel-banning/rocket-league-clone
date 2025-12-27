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
        }
    }
}

bool Match::ballCollides(const physics::Plane &plane)
{
    // find closes point on the aabb to the sphere center
    glm::vec3 closest;
    closest.x = glm::clamp(ball.position.x, plane.getMin().x, plane.getMax().x);
    return false;
}
} // namespace engine
