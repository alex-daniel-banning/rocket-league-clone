#define GLM_ENABLE_EXPERIMENTAL
#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <cmath>
#include <engine/match.hpp>
#include <engine/path_manager.hpp>
#include <engine/physics/box.hpp>
#include <engine/physics/box_builder.hpp>
#include <engine/physics/plane.hpp>
#include <engine/render/camera.hpp>
#include <engine/render/renderer.hpp>
#include <engine/render/shader.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/dual_quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

#define GL_CHECK()                                                                                              \
  do {                                                                                                          \
    GLenum err;                                                                                                 \
    while ((err = glGetError()) != GL_NO_ERROR) {                                                               \
      std::cerr << "OpenGL error at " << __FILE__ << ":" << __LINE__ << " -> 0x" << std::hex << err << std::dec \
                << '\n';                                                                                        \
    }                                                                                                           \
  } while (0)

unsigned int LoadTexture(const char* path);

float last_x, last_y;
bool first_mouse = true;
int mouse_callback_count = 0;
void MouseCallback(GLFWwindow* window, double xpos, double ypos);
void ProcessInput(GLFWwindow* window, engine::render::Camera& camera);

float delta_time = 0.0f;  // Time between current frame and last frame
float last_frame = 0.0f;  // Time of last frame

const unsigned int shadow_width = 1024, shadow_height = 1024;
const float light_projection_near_plane = 1.0f, light_projection_far_plane = 70.0f;
unsigned int depth_map_fbo;
unsigned int depth_map;
void SetupShaderBuffer();

int main() {
  // Initialize GLFW
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return -1;
  }

  // Tell GLFW what kind of context to create
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWmonitor* primary = glfwGetPrimaryMonitor();
  const GLFWvidmode* mode = glfwGetVideoMode(primary);
  float screen_width = mode->width;
  float screen_height = mode->height;
  last_x = screen_width / 2.0f;
  last_y = screen_height / 2.0f;

  GLFWwindow* window = glfwCreateWindow(screen_width, screen_height, "Fullscreen Window", primary, nullptr);

  if (!window) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return -1;
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  glfwSetCursorPos(window, screen_width / 2.0f, screen_height / 2.0f);
  glfwSetCursorPosCallback(window, MouseCallback);
  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

  // Initialize glad
  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    std::cerr << "Failed to initialize GLAD\n";
    return -1;
  }
  // Setup completed.

  glEnable(GL_DEPTH_TEST);
  engine::render::Camera camera(glm::vec3(24.0f, 10.0f, 24.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f);
  camera.LookAt(glm::vec3(0.0f, 5.0f, 0.0f));
  camera.projection.far_plane = 300.0f;
  glfwSetWindowUserPointer(window, &camera);

  SetupShaderBuffer();

  glm::vec3 light_pos(0.0f, 40.0f, 0.0f);
  engine::render::Shader simple_depth_shader(
      engine::PathManager::GlobalAsset("shaders/simple_depth_shader.vert").c_str(),
      engine::PathManager::GlobalAsset("shaders/empty_shader.frag").c_str());

  // create regular shader
  engine::render::Shader model_loading_shader(engine::PathManager::GlobalAsset("shaders/model_loading.vert").c_str(),
                                              engine::PathManager::GlobalAsset("shaders/model_loading.frag").c_str());

  engine::render::Renderer renderer(screen_width, screen_height);

  float ground_size = 50.0f;
  glm::vec3 wall_color(0.2f, 0.15f, 0.15f);
  // Why does it not matter that my rotation is 90 or -90? i.e. I can normal
  // isn't behaving as expected.
  float wall_translation_size = ground_size / 2;
  std::vector<engine::physics::Box> walls = {
      engine::physics::BoxBuilder()
          .Size(glm::vec3(ground_size, 1.0f, ground_size))
          .Position(glm::vec3(wall_translation_size, wall_translation_size, 0.0f))
          .Rotation(glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)))
          .Mass(0.0f)
          .Build(),
      engine::physics::BoxBuilder()
          .Size(glm::vec3(ground_size, 1.0f, ground_size))
          .Position(glm::vec3(-wall_translation_size, wall_translation_size, 0.0f))
          .Rotation(glm::angleAxis(glm::radians(-90.0f), glm::vec3(0.0f, 0.0f, 1.0f)))
          .Mass(0.0f)
          .Build(),
      engine::physics::BoxBuilder()
          .Size(glm::vec3(ground_size, 1.0f, ground_size))
          .Position(glm::vec3(0.0f, wall_translation_size, wall_translation_size))
          .Rotation(glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f)))
          .Mass(0.0f)
          .Build(),
      engine::physics::BoxBuilder()
          .Size(glm::vec3(ground_size, 1.0f, ground_size))
          .Position(glm::vec3(0.0f, wall_translation_size, -wall_translation_size))
          .Rotation(glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)))
          .Mass(0.0f)
          .Build()};

  engine::Match match =
      engine::Match::Builder()
          .WithBall(engine::physics::Sphere(1.0f, 10.0f, glm::vec3(10.0f, 4.5f, 0.0f), glm::vec3(-5.0f, 0.0f, 0.0f)))
          .WithGround(engine::physics::BoxBuilder()
                          .Size(glm::vec3(ground_size, 1.0f, ground_size))
                          .Position(glm::vec3(0.0f, -0.5f, 0.0f))
                          .Mass(0.0f)
                          .Build())
          //.WithWalls(walls)
          .WithBox(engine::physics::Box(glm::vec3(5.0f), glm::vec3(0.0f, 3.0f, 0.0f), glm::vec3(0.0f), 30.0f,
                                        glm::quat(), glm::vec3(0.0f, 0.0f, 0.0f)))
          .Build();

  float demo_start = glfwGetTime();
  // Main loop
  while (!glfwWindowShouldClose(window)) {
    float current_frame = glfwGetTime();
    if (current_frame - demo_start > 3.0) {
      demo_start = glfwGetTime();
      match.Reset();
    }
    delta_time = current_frame - last_frame;
    last_frame = current_frame;
    ProcessInput(window, camera);
    match.Tick(delta_time);

    // render
    // ------
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 1. render depth of scene to texture (from light's perspective)
    // --------------------------------------------------------------
    glm::mat4 light_projection, light_view;
    glm::mat4 light_space_matrix;
    light_projection = glm::ortho(-ground_size, ground_size, -ground_size, ground_size, light_projection_near_plane,
                                  light_projection_far_plane);
    glViewport(0, 0, shadow_width, shadow_height);
    glBindFramebuffer(GL_FRAMEBUFFER, depth_map_fbo);
    glClear(GL_DEPTH_BUFFER_BIT);

    glm::vec3 light_dir = glm::normalize(glm::vec3(0.0f) - light_pos);
    glm::vec3 up_vector = glm::vec3(0.0f, 1.0f, 0.0f);
    // If light is pointing nearly straight up or down, use a different vector
    if (std::abs(light_dir.y) > 0.99f) {
      up_vector = glm::vec3(0.0f, 0.0f, 1.0f);
    }
    light_view = glm::lookAt(light_pos, glm::vec3(0.0f), up_vector);

    light_space_matrix = light_projection * light_view;
    simple_depth_shader.Use();
    simple_depth_shader.SetMat4("lightSpaceMatrix", light_space_matrix);

    simple_depth_shader.Use();
    if (match.GetGround()) {
      renderer.DrawBox(*match.GetGround(), simple_depth_shader);
    }
    if (match.GetBall()) {
      renderer.DrawSphere(*match.GetBall(), simple_depth_shader);
    }
    for (engine::physics::Box box : match.GetBoxes()) {
      renderer.DrawBox(box, simple_depth_shader);
    }

    GL_CHECK();

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // reset viewport
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, mode->width, mode->height);

    // Render Floor Model
    model_loading_shader.Use();
    model_loading_shader.SetMat4("lightSpaceMatrix", light_space_matrix);
    model_loading_shader.SetInt("shadowMap", 8);
    glActiveTexture(GL_TEXTURE8);  // todo set to 8 so it doesn't conflict with model
                                   // texture index, need a better long term solution
    glBindTexture(GL_TEXTURE_2D, depth_map);
    model_loading_shader.SetBool("useTexture", false);
    model_loading_shader.SetVec3("lightColor", glm::vec3(1.0f));
    model_loading_shader.SetVec3("lightPos", light_pos);
    model_loading_shader.SetVec3("viewPos", camera.position);
    if (match.GetGround()) {
      model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.2f));
      renderer.DrawBox(*match.GetGround(), model_loading_shader, camera);
    }
    if (!match.GetWalls().empty()) model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.3f));
    for (engine::physics::Box wall : match.GetWalls()) {
      renderer.DrawBox(wall, model_loading_shader, camera);
    }
    if (match.GetBall()) {
      model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.0f, 0.5f, 0.3f));
      renderer.DrawSphere(*match.GetBall(), model_loading_shader, camera);
    }
    if (!match.GetBoxes().empty()) model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.0f, 0.5f, 0.3f));
    for (engine::physics::Box box : match.GetBoxes()) {
      renderer.DrawBox(box, model_loading_shader, camera);
    }

    // debug rendering
    for (engine::physics::Box wall : match.GetWalls()) {
      renderer.DrawBoxWireframe(wall, model_loading_shader, camera);
    }
    if (match.GetGround()) {
      renderer.DrawBoxWireframe(*match.GetGround(), model_loading_shader, camera);
    }
    for (engine::physics::Box box : match.GetBoxes()) {
      renderer.DrawBoxWireframe(box, model_loading_shader, camera);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // Clean up
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}

void SetupShaderBuffer() {
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

void ProcessInput(GLFWwindow* window, engine::render::Camera& camera) {
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

  // make sure that when camera.Pitch is out of bounds, screen doesn't get
  // flipped
  if (camera.pitch > 89.0f) camera.pitch = 89.0f;
  if (camera.pitch < -89.0f) camera.pitch = -89.0f;

  glm::vec3 front;
  front.x = cos(glm::radians(camera.yaw)) * cos(glm::radians(camera.pitch));
  front.y = sin(glm::radians(camera.pitch));
  front.z = sin(glm::radians(camera.yaw)) * cos(glm::radians(camera.pitch));
  camera.front = glm::normalize(front);
}

void MouseCallback(GLFWwindow* window, double x_pos_in, double y_pos_in) {
  auto* camera = static_cast<engine::render::Camera*>(glfwGetWindowUserPointer(window));
  float mouse_sensitivity = 0.05f;
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
  float yoffset = last_y - ypos;  // reversed since y-coordinates go from bottom to top

  last_x = xpos;
  last_y = ypos;

  assert(camera && "Camera pointer not set via glfwSetWindowUserPointer!");
  camera->ProcessMouseMovement(xoffset, yoffset, mouse_sensitivity);
}
