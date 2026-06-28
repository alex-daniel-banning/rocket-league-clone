#include "engine/match.hpp"

#include "engine/log.hpp"
#include "engine/physics/box.hpp"
#include "engine/physics/collisions.hpp"
#include "engine/physics/constraint_solver.hpp"
#include "engine/physics/inertia.hpp"

static constexpr glm::vec3 gravity = glm::vec3(0.0f, -9.8f, 0.0f);

namespace engine {

void Match::Tick(float delta_time) {
  accumulator_ += delta_time;
  int substeps = 0;
  while (accumulator_ >= fixed_dt) {
    substeps++;
    ApplyGravity();

    // --- Driving forces ---
    for (Car& car : cars_) car.AccumulateDrivingForces(ResolveChassis(car));  // fills force_/torque_accumulator
    IntegrateForces();                                                        // v += F * mass_inv * dt
                                                                              // w += I-1_world dot torque dot dt

    // --- Velocity constraint solving and integration ---
    auto constraints = GenerateContactConstraints(fixed_dt);
    constraint_solver_.PreSolve(bodies_, constraints.normal, constraints.friction, fixed_dt);
    constexpr int solver_iterations = 10;
    constraint_solver_.Solve(bodies_, constraints.normal, constraints.friction, solver_iterations);
    if (ball_) {
      ball_->position += fixed_dt * ball_->EffectiveVelocity();
      glm::quat q = ball_->rotation;
      glm::vec3 w = ball_->EffectiveAngularVelocity();
      ball_->rotation = glm::normalize(q + (0.5f * fixed_dt * glm::quat(0, w.x, w.y, w.z) * q));
    }
    for (physics::Box& car : boxes_) {
      car.position += fixed_dt * car.EffectiveVelocity();
      glm::quat q = car.rotation;
      glm::vec3 w = car.EffectiveAngularVelocity();
      car.rotation = glm::normalize(q + (0.5f * fixed_dt * glm::quat(0, w.x, w.y, w.z) * q));
    }

    // --- Damping ---
    const float linear_damping_per_sec = 0.98f;
    const float angular_damping_per_sec = 0.98f;
    float linear_damp = std::pow(linear_damping_per_sec, fixed_dt);
    float angular_damp = std::pow(angular_damping_per_sec, fixed_dt);
    if (ball_) {
      ball_->velocity *= linear_damp;
    }
    for (physics::Box& car : boxes_) {
      car.velocity *= linear_damp;
      car.angular_velocity *= angular_damp;
    }

    accumulator_ -= fixed_dt;
  }
  if (substeps > 10) {
    LOG_WARN("tick death spiral: %d substeps (dt=%.4f)", substeps, delta_time);
  }
  // LOG_TRACE("tick: dt=%.4f substeps=%d", delta_time, substeps);
}

void Match::Reset() {
  if (ball_ && initial_ball_) ball_.emplace(*initial_ball_);
  boxes_ = {initial_boxes_.begin(), initial_boxes_.end()};
}

void Match::ApplyGravity() {
  // Gravity as a force (F = m*g) so IntegrateForces is the single integration
  // point; a = F * mass_inv = g recovers the same acceleration.
  for (physics::Box& box : boxes_) box.force_accumulator += box.mass * gravity;
  if (ball_) ball_->force_accumulator += ball_->mass * gravity;
}

void Match::IntegrateForces() {
  // Apply accumulated force/torque to velocity, then clear the accumulators so
  // the next tick starts clean. Box and Sphere expose the same fields.
  auto integrate = [](auto& b) {
    b.velocity += (b.force_accumulator * b.mass_inv) * fixed_dt;
    b.angular_velocity +=
        physics::WorldInverseInertia(b.rotation, b.inertia_tensor_inv) * b.torque_accumulator * fixed_dt;
    b.force_accumulator = glm::vec3(0.0f);
    b.torque_accumulator = glm::vec3(0.0f);
  };
  for (physics::Box& box : boxes_) integrate(box);
  if (ball_) integrate(*ball_);
}

Match::ContactConstraints Match::GenerateContactConstraints(float dt) {
  ContactConstraints constraints;

  const float baumgarte = 0.02f;
  const float restitution = 0.3f;

  // Detect a contact between a and b and, if present, generate its constraints.
  // ComputeContact's overloads expect (Box, Sphere) or (Box, Box), so a must be
  // a Box; the ball is always passed as b.
  auto try_pair = [&](const auto& a, const auto& b) {
    physics::Contact contact;
    if (physics::collisions::ComputeContact(a, b, contact)) {
      physics::ConstraintSolver::GenerateFromContact(contact, bodies_, dt, constraints.normal, constraints.friction,
                                                     restitution, baumgarte);
    }
  };

  if (ball_) {
    for (const physics::Box& wall : walls_) try_pair(wall, *ball_);  // Ball v Wall
    if (ground_) try_pair(*ground_, *ball_);                         // Ball v Ground
    for (const physics::Box& car : boxes_) try_pair(car, *ball_);    // Ball v Car
  }

  // Car v Car
  for (std::size_t i = 0; i + 1 < boxes_.size(); i++) {
    for (std::size_t j = i + 1; j < boxes_.size(); j++) try_pair(boxes_[i], boxes_[j]);
  }
  // Car v Wall
  for (const physics::Box& car : boxes_) {
    for (const physics::Box& wall : walls_) try_pair(car, wall);
  }
  // Car v Ground
  if (ground_) {
    for (const physics::Box& car : boxes_) try_pair(car, *ground_);
  }

  return constraints;
}

void Match::SetCarInput(CarInput car_input) {
  // TODO: single-player assumption — routes input to the first car only.
  // Generalize (car/player id in the signature) when multiple cars exist.
  if (!cars_.empty()) cars_[0].SetInput(car_input);
}

}  // namespace engine
