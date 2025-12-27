#include <engine/Match.hpp>

namespace engine
{

void Match::tick(float deltaTime) { ball.position += deltaTime * ball.velocity; }

} // namespace engine
