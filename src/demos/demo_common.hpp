#pragma once

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <cassert>
#include <cmath>
#include <iostream>

#include "engine/render/camera.hpp"

// ---- Globals ----------------------------------------------------------------

inline float delta_time = 0.0f;
inline float last_frame = 0.0f;
inline float last_x = 0.0f;
inline float last_y = 0.0f;
inline bool first_mouse = true;
inline int mouse_callback_count = 0;

inline constexpr unsigned int shadow_width = 1024;
inline constexpr unsigned int shadow_height = 1024;
inline constexpr float light_projection_near_plane = 1.0f;
inline constexpr float light_projection_far_plane = 70.0f;
inline unsigned int depth_map_fbo = 0;
inline unsigned int depth_map = 0;

// ---- Input ------------------------------------------------------------------

inline void ProcessInput(GLFWwindow* window, engine::render::Camera& camera) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(window, true);

  const float camera_speed = camera.movement_speed * delta_time;
  glm::vec3 forward = glm::normalize(glm::vec3(camera.front.x, 0.0f, camera.front.z));

  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) camera.position += camera_speed * forward;
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) camera.position -= camera_speed * forward;
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    camera.position -= glm::normalize(glm::cross(camera.front, camera.up)) * camera_speed;
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    camera.position += glm::normalize(glm::cross(camera.front, camera.up)) * camera_speed;
  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) camera.position += glm::vec3(0.0f, 1.0f, 0.0f) * camera_speed;
  if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    camera.position -= glm::vec3(0.0f, 1.0f, 0.0f) * camera_speed;

  if (camera.pitch > 89.0f) camera.pitch = 89.0f;
  if (camera.pitch < -89.0f) camera.pitch = -89.0f;

  glm::vec3 front;
  front.x = cos(glm::radians(camera.yaw)) * cos(glm::radians(camera.pitch));
  front.y = sin(glm::radians(camera.pitch));
  front.z = sin(glm::radians(camera.yaw)) * cos(glm::radians(camera.pitch));
  camera.front = glm::normalize(front);
}

inline void MouseCallback(GLFWwindow* window, double x_pos_in, double y_pos_in) {
  auto* camera = static_cast<engine::render::Camera*>(glfwGetWindowUserPointer(window));
  constexpr float mouse_sensitivity = 0.05f;
  float xpos = static_cast<float>(x_pos_in);
  float ypos = static_cast<float>(y_pos_in);

  if (mouse_callback_count < 2) {
    last_x = xpos;
    last_y = ypos;
    mouse_callback_count++;
    return;
  }
  if (first_mouse) {
    last_x = xpos;
    last_y = ypos;
    first_mouse = false;
    return;
  }

  float xoffset = xpos - last_x;
  float yoffset = last_y - ypos;
  last_x = xpos;
  last_y = ypos;

  assert(camera && "Camera pointer not set via glfwSetWindowUserPointer!");
  camera->ProcessMouseMovement(xoffset, yoffset, mouse_sensitivity);
}

// ---- Window init ------------------------------------------------------------

struct DemoWindow {
  GLFWwindow* window;
  const GLFWvidmode* mode;
  float width;
  float height;
};

inline DemoWindow InitWindow(const char* title = "Demo") {
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    std::exit(-1);
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWmonitor* primary = glfwGetPrimaryMonitor();
  const GLFWvidmode* mode = glfwGetVideoMode(primary);
  float width = static_cast<float>(mode->width);
  float height = static_cast<float>(mode->height);
  last_x = width / 2.0f;
  last_y = height / 2.0f;

  GLFWwindow* window = glfwCreateWindow(static_cast<int>(width), static_cast<int>(height), title, primary, nullptr);
  if (!window) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    std::exit(-1);
  }

  glfwMakeContextCurrent(window);
  glfwSetCursorPos(window, width / 2.0f, height / 2.0f);
  glfwSetCursorPosCallback(window, MouseCallback);
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    std::cerr << "Failed to initialize GLAD\n";
    std::exit(-1);
  }

  return {window, mode, width, height};
}

// ---- Shadow buffer ----------------------------------------------------------

inline void SetupShadowBuffer() {
  glGenFramebuffers(1, &depth_map_fbo);
  glGenTextures(1, &depth_map);
  glBindTexture(GL_TEXTURE_2D, depth_map);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, shadow_width, shadow_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT,
               nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindFramebuffer(GL_FRAMEBUFFER, depth_map_fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_map, 0);
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
