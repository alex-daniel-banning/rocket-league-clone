#include <engine/Match.hpp>
#include <engine/physics/Collisions.hpp>

namespace engine
{

void Match::tick(float deltaTime)
{
    ball.position += deltaTime * ball.velocity;
    handleCollisions();
}

void Match::reset()
{
    ball  = initialBall;
    boxes = initialBoxes;
}

void Match::handleCollisions()
{
    for (physics::Plane wall : walls)
    {
        if (physics::Collisions::collides(wall, ball))
        {
            physics::Collisions::handleElasticCollision(wall, ball);
        }
    }
    if (physics::Collisions::collides(ground, ball))
    {
        physics::Collisions::handleElasticCollision(ground, ball);
    }
}

} // namespace engine
