#include <engine/Match.hpp>
#include <engine/physics/Collisions.hpp>
#include <iostream>

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
        physics::Collisions::handleElasticCollision(wall, ball);
    }
    physics::Collisions::handleElasticCollision(ground, ball);

    for (physics::Box box : boxes)
    {

        physics::Collisions::handleElasticCollision(box, ball);
    }
}

} // namespace engine
