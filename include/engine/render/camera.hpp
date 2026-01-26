#pragma once

#include <glm/glm.hpp>

namespace engine::render {

// Defines several possible options for camera movement. Used as abstraction to
// stay away from window-system specific input methods
enum Camera_Movement { FORWARD, BACKWARD, LEFT, RIGHT };

struct ProjectionSettings {
  float fov = 45.0f;
  float near_plane = 0.1f;
  float far_plane = 100.0f;
};

// Default camera values
const float default_yaw = -90.0f;
const float default_pitch = 0.0f;
const float default_speed = 15.0f;
const float default_sensitivity = 4.0f;
const float default_zoom = 45.0f;

// An abstract camera class that processes input and calculates the
// corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera {
 public:
  // camera Attributes
  glm::vec3 position;
  glm::vec3 front;
  glm::vec3 up;
  glm::vec3 world_up;
  // euler Angles
  float yaw;
  float pitch;
  // camera options
  float movement_speed;
  float zoom;
  float sensitivity;
  float near_plane, far_plane;
  ProjectionSettings projection;

  // constructor with vectors
  Camera(glm::vec3 position, glm::vec3 up, float yaw = default_yaw,
         float pitch = default_pitch);

  // returns the view matrix calculated using Euler Angles and the LookAt Matrix
  glm::mat4 GetViewMatrix() const;

  // processes input received from any keyboard-like input system. Accepts input
  // parameter in the form of camera defined ENUM (to abstract it from windowing
  // systems)
  void ProcessKeyboard(Camera_Movement direction, float delta_time);
  void ProcessMouseMovement(float xoffset, float yoffset,
                            float mouse_sensitivity,
                            bool constrain_pitch = true);

  void LookAtOrigin();
  void LookAt(glm::vec3 target);

 private:
  glm::vec3 right_;
  // calculates the front vector from the Camera's (updated) Euler Angles
  void UpdateCameraVectors();
};
}  // namespace engine::render
