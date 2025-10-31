#include "Camera.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// constructor with vectors
Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch)
    : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), Zoom(ZOOM),
      Sensitivity(SENSITIVITY)
{
    Position = position;
    WorldUp  = up;
    Yaw      = yaw;
    Pitch    = pitch;
    updateCameraVectors();
};

// constructor with scalar values
Camera::Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw,
               float pitch)
    : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), Zoom(ZOOM),
      Sensitivity(SENSITIVITY)
{
    Position = glm::vec3(posX, posY, posZ);
    WorldUp  = glm::vec3(upX, upY, upZ);
    Yaw      = yaw;
    Pitch    = pitch;
    updateCameraVectors();
};

// returns the view matrix calculated using Euler Angles and the LookAt Matrix
glm::mat4 Camera::GetViewMatrix() { return glm::lookAt(Position, Position + Front, Up); };

// processes input received from any keyboard-like input system. Accepts input parameter in the
// form of camera defined ENUM (to abstract it from windowing systems)
void Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime)
{
    glm::vec3 forward = glm::normalize(glm::vec3(Front.x, 0.0f, Front.z));
    float velocity    = MovementSpeed * deltaTime;
    if (direction == FORWARD)
        Position += forward * velocity;
    if (direction == BACKWARD)
        Position -= forward * velocity;
    if (direction == LEFT)
        Position -= Right * velocity;
    if (direction == RIGHT)
        Position += Right * velocity;
};

// calculates the front vector from the Camera's (updated) Euler Angles
void Camera::updateCameraVectors()
{
    // calculate the new Front vector
    glm::vec3 front;
    front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front.y = sin(glm::radians(Pitch));
    front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    Front   = glm::normalize(front);
    // also re-calculate the Right and Up vector
    Right = glm::normalize(glm::cross(
        Front, WorldUp)); // normalize the vectors, because their length gets closer to 0 the
                          // more you look up or down which results in slower movement.
    Up = glm::normalize(glm::cross(Right, Front));
};
