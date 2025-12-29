#include <engine/Match.hpp>
#include <gtest/gtest.h>

TEST(MatchTest, BasicCollisionDetection)
{
    glm::vec3 color(0.2f, 0.15f, 0.15f);

    float groundSize = 10.0f;
    engine::physics::Plane ground(groundSize, groundSize, color);

    float wallTranslationSize                 = groundSize / 2;
    std::vector<engine::physics::Plane> walls = {
        engine::physics::Plane(groundSize, groundSize, color,
                               glm::vec3(wallTranslationSize, wallTranslationSize, 0.0f),
                               glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f))),
        engine::physics::Plane(groundSize, groundSize, color,
                               glm::vec3(-wallTranslationSize, wallTranslationSize, 0.0f),
                               glm::angleAxis(glm::radians(-90.0f), glm::vec3(0.0f, 0.0f, 1.0f))),
        engine::physics::Plane(groundSize, groundSize, color,
                               glm::vec3(0.0f, wallTranslationSize, wallTranslationSize),
                               glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f))),
        engine::physics::Plane(groundSize, groundSize, color,
                               glm::vec3(0.0f, wallTranslationSize, -wallTranslationSize),
                               glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)))};
    engine::physics::Sphere ball;
    ball.radius   = 1.0f;
    ball.position = glm::vec3(0.0f, 20.0f, 1.0f);
    ball.velocity = glm::vec3(0.0f, 0.0f, 0.0f);

    engine::Match match(ball, ground, walls);
}
