#pragma once
#include <engine/physics/box.hpp>

namespace engine::physics {
class BoxBuilder {
 public:
  BoxBuilder& Size(glm::vec3 s) {
    size_ = s;
    return *this;
  }
  BoxBuilder& Size(float s) {
    size_ = glm::vec3(s);
    return *this;
  }
  BoxBuilder& Position(glm::vec3 p) {
    position_ = p;
    return *this;
  }
  BoxBuilder& Position(float x, float y, float z) {
    position_ = glm::vec3(x, y, z);
    return *this;
  }
  BoxBuilder& Velocity(glm::vec3 v) {
    velocity_ = v;
    return *this;
  }
  BoxBuilder& Velocity(float x, float y, float z) {
    velocity_ = glm::vec3(x, y, z);
    return *this;
  }
  BoxBuilder& Mass(float m) {
    mass_ = m;
    return *this;
  }
  BoxBuilder& Rotation(glm::quat r) {
    rotation_ = r;
    return *this;
  }
  BoxBuilder& AngularVelocity(glm::vec3 w) {
    angular_velocity_ = w;
    return *this;
  }
  BoxBuilder& AngularVelocity(float x, float y, float z) {
    angular_velocity_ = glm::vec3(x, y, z);
    return *this;
  }

  engine::physics::Box Build() const {
    return engine::physics::Box(size_, position_, velocity_, mass_, rotation_, angular_velocity_);
  }

 private:
  glm::vec3 size_ = glm::vec3(1.0f);
  glm::vec3 position_ = glm::vec3(0.0f);
  glm::vec3 velocity_ = glm::vec3(0.0f);
  float mass_ = 1.0f;
  glm::quat rotation_ = glm::quat(glm::vec3(0.0f));
  glm::vec3 angular_velocity_ = glm::vec3(0.0f);
};
}  // namespace engine::physics
