#include <Print.hpp>
#include <engine/Match.hpp>
#include <engine/physics/Box.hpp>
#include <engine/physics/Collisions.hpp>
#include <engine/physics/Contact.hpp>

namespace engine {

void Match::tick(float deltaTime) {
  accumulator += deltaTime;
  while (accumulator >= FIXED_DT) {
    // Define how much damping PER SECOND
    const float LINEAR_DAMPING_PER_SEC = 0.98f;
    const float ANGULAR_DAMPING_PER_SEC = 0.95f;

    // Convert to per-frame damping
    float linear_damp = std::pow(LINEAR_DAMPING_PER_SEC, FIXED_DT);
    float angular_damp = std::pow(ANGULAR_DAMPING_PER_SEC, FIXED_DT);

    ball.position += FIXED_DT * ball.velocity;
    // damping
    ball.velocity *= linear_damp;
    for (physics::Box &box : boxes) {
      box.position += FIXED_DT * box.velocity;
      glm::quat q = box.rotation;
      glm::vec3 w = box.angular_velocity;
      box.rotation = glm::normalize(
          q + (0.5f * FIXED_DT * q * glm::quat(0, w.x, w.y, w.z)));
      box.velocity *= linear_damp;
      box.angular_velocity *= angular_damp;
    }
    handleCollisions();
    accumulator -= FIXED_DT;
  }
}

void Match::reset() {
  ball = initialBall;
  boxes = initialBoxes;
}

void Match::handleCollisions() {
  for (physics::Plane wall : walls) {
    physics::Collisions::handleElasticCollision(wall, ball);
  }
  physics::Collisions::handleElasticCollision(ground, ball);

  for (physics::Box &box : boxes) {
    physics::Contact contact;
    if (physics::Collisions::computeContact(box, ball, contact)) {
      physics::Collisions::resolveElasticCollision(box, ball, contact);
    }
  }
}

}  // namespace engine
