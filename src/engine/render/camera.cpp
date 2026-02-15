#include <engine/render/camera.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace engine::render {

// constructor with vectors
Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch)
    : front(glm::vec3(0.0f, 0.0f, -1.0f)),
      movement_speed(default_speed),
      zoom(default_zoom),
      sensitivity(default_sensitivity),
      position(position),
      world_up(up),
      yaw(yaw),
      pitch(pitch) {
  UpdateCameraVectors();
};

// returns the view matrix calculated using Euler Angles and the LookAt Matrix
glm::mat4 Camera::GetViewMatrix() const { return glm::lookAt(position, position + front, up); };

// processes input received from any keyboard-like input system. Accepts input
// parameter in the form of camera defined ENUM (to abstract it from windowing
// systems)
void Camera::ProcessKeyboard(Camera_Movement direction, float delta_time) {
  glm::vec3 forward = glm::normalize(glm::vec3(front.x, 0.0f, front.z));
  float velocity = movement_speed * delta_time;
  if (direction == FORWARD) position += forward * velocity;
  if (direction == BACKWARD) position -= forward * velocity;
  if (direction == LEFT) position -= right_ * velocity;
  if (direction == RIGHT) position += right_ * velocity;
};

void Camera::ProcessMouseMovement(float xoffset, float yoffset, const float mouse_sensitivity, bool constrain_pitch) {
  xoffset *= mouse_sensitivity;
  yoffset *= mouse_sensitivity;

  yaw += xoffset;
  pitch += yoffset;

  // make sure that when pitch is out of bounds, screen doesn't get flipped
  if (constrain_pitch) {
    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;
  }

  // update Front, Right and Up Vectors using the updated Euler angles
  UpdateCameraVectors();
}

// calculates the front vector from the Camera's (updated) Euler Angles
void Camera::UpdateCameraVectors() {
  // calculate the new Front vector
  glm::vec3 front;
  front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
  front.y = sin(glm::radians(pitch));
  front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
  front = glm::normalize(front);

  // Assert that Front is not parallel to WorldUp (gimbal lock check)
  float dot_product = std::abs(glm::dot(front, glm::normalize(world_up)));
  assert(dot_product < 0.9999f && "Camera Front Vector is parallel to WorldUp - gimbal lock!");

  // also re-calculate the Right and Up vector
  right_ = glm::normalize(glm::cross(front, world_up));  // normalize the vectors, because their length gets
                                                         // closer to 0 the more you look up or down which
                                                         // results in slower movement.
  assert(glm::length(glm::cross(front, world_up)) > 0.0001f &&
         "Right vector has near-zero length - camera looking straight up/down!");

  up = glm::normalize(glm::cross(right_, front));
};

void Camera::LookAtOrigin() {
  front = glm::normalize(glm::vec3(0.0f) - position);
  right_ = glm::normalize(glm::cross(front, world_up));  // normalize the vectors, because their length gets
                                                         // closer to 0 the more you look up or down which
                                                         // results in slower movement.
  up = glm::normalize(glm::cross(right_, front));
  pitch = glm::degrees(std::asin(front.y));
  yaw = glm::degrees(atan2(front.z, front.x));
}

void Camera::LookAt(glm::vec3 target)

{
  front = glm::normalize(target - position);
  right_ = glm::normalize(glm::cross(front, world_up));  // normalize the vectors, because their length gets
                                                         // closer to 0 the more you look up or down which
                                                         // results in slower movement.
  up = glm::normalize(glm::cross(right_, front));
  pitch = glm::degrees(std::asin(front.y));
  yaw = glm::degrees(atan2(front.z, front.x));
}
}  // namespace engine::render
