#include <engine/Match.hpp>
#include <engine/physics/Box.hpp>
#include <engine/physics/Collisions.hpp>
#include <engine/physics/Contact.hpp>

namespace engine
{

void Match::tick(float deltaTime)
{
    ball.position += deltaTime * ball.velocity;
    for (physics::Box &box : boxes)
    {
        box.position += deltaTime * box.velocity;
    }
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

    for (physics::Box &box : boxes)
    {
        physics::Contact contact;
        if (physics::Collisions::computeContact(box, ball, contact))
        {
            physics::Collisions::resolveElasticCollision(box, ball, contact);
        }
    }
}

} // namespace engine
