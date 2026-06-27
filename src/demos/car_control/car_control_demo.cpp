#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

#include "../demo_common.hpp"
#include "engine/match.hpp"
#include "engine/path_manager.hpp"
#include "engine/physics/box.hpp"
#include "engine/physics/box_builder.hpp"
#include "engine/physics/sphere.hpp"
#include "engine/render/renderer.hpp"
#include "engine/render/shader.hpp"

#define GL_CHECK()                                                                                              \
  do {                                                                                                          \
    GLenum err;                                                                                                 \
    while ((err = glGetError()) != GL_NO_ERROR) {                                                               \
      std::cerr << "OpenGL error at " << __FILE__ << ":" << __LINE__ << " -> 0x" << std::hex << err << std::dec \
                << '\n';                                                                                        \
    }                                                                                                           \
  } while (0)

const std::unordered_map<std::string, glm::vec3> name_to_color = {{"Red Box", glm::vec3(1.0f, 0.0f, 0.0f)}};

// --- Car Input ---
float throttle = 0.0f;
float steering = 0.0f;

// ---- Input ------------------------------------------------------------------

inline void ProcessCarInput(GLFWwindow* window) {
  throttle = 0.0f;
  steering = 0.0f;
  if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) throttle = 1.0f;
  if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) throttle = -1.0f;
  if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) steering = 1.0f;
  if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) steering = -1.0f;
}

int main() {
  const float room_size = 200.0f;
  const float half = room_size / 2.0f;

  engine::Match match =
      engine::Match::Builder()
          .WithGround(engine::physics::BoxBuilder()
                          .Size(glm::vec3(room_size, 1.0f, room_size))
                          .Position(glm::vec3(0.0f, 0.5f, 0.0f))
                          .Mass(0.0f)
                          .Name("Ground")
                          .Build())
          .WithWall(engine::physics::BoxBuilder()
                        .Size(glm::vec3(room_size, 1.0f, room_size))
                        .Position(glm::vec3(0.0f, room_size + 0.5f, 0.0f))
                        .Mass(0.0f)
                        .Name("Ceiling")
                        .Build())
          .WithWall(engine::physics::BoxBuilder()
                        .Size(glm::vec3(room_size, 1.0f, room_size))
                        .Position(glm::vec3(half, half, 0.0f))
                        .Rotation(glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)))
                        .Mass(0.0f)
                        .Name("Wall (+X)")
                        .Build())
          .WithWall(engine::physics::BoxBuilder()
                        .Size(glm::vec3(room_size, 1.0f, room_size))
                        .Position(glm::vec3(-half, half, 0.0f))
                        .Rotation(glm::angleAxis(glm::radians(-90.0f), glm::vec3(0.0f, 0.0f, 1.0f)))
                        .Mass(0.0f)
                        .Name("Wall (-X)")
                        .Build())
          .WithWall(engine::physics::BoxBuilder()
                        .Size(glm::vec3(room_size, 1.0f, room_size))
                        .Position(glm::vec3(0.0f, half, half))
                        .Rotation(glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f)))
                        .Mass(0.0f)
                        .Name("Wall (+Z)")
                        .Build())
          .WithWall(engine::physics::BoxBuilder()
                        .Size(glm::vec3(room_size, 1.0f, room_size))
                        .Position(glm::vec3(0.0f, half, -half))
                        .Rotation(glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)))
                        .Mass(0.0f)
                        .Name("Wall (-Z)")
                        .Build())
          //.WithBall(engine::physics::Sphere(1.0f, 5.0f, glm::vec3(0.0f, 10.0f, 0.0f), glm::vec3(3.0f, -2.0f, 4.0f)))
          .WithBox(engine::physics::BoxBuilder()
                       .Size(glm::vec3(2.0f, 1.5f, 5.0f))
                       .Position(glm::vec3(-6.0f, 5.0f, 4.0f))
                       .Mass(15.0f)
                       .Velocity(glm::vec3())
                       .AngularVelocity(glm::vec3())
                       .Name("Red Box")
                       .Build())
          .Build();

  auto [window, mode, screen_width, screen_height] = InitWindow("perf demo");
  glEnable(GL_DEPTH_TEST);

  engine::render::Model car(engine::PathManager::GlobalAsset("car/car.obj").c_str());

  engine::render::Camera camera(glm::vec3(30.0f, 20.0f, 30.0f), glm::vec3(0.0f, 1.0f, 0.0f), -135.0f, -20.0f);
  camera.projection.far_plane = 300.0f;
  glfwSetWindowUserPointer(window, &camera);

  SetupShadowBuffer();

  glm::vec3 light_pos(0.0f, 40.0f, 0.0f);
  engine::render::Shader simple_depth_shader(
      engine::PathManager::GlobalAsset("shaders/simple_depth_shader.vert").c_str(),
      engine::PathManager::GlobalAsset("shaders/empty_shader.frag").c_str());
  engine::render::Shader model_loading_shader(engine::PathManager::GlobalAsset("shaders/model_loading.vert").c_str(),
                                              engine::PathManager::GlobalAsset("shaders/model_loading.frag").c_str());
  engine::render::Renderer renderer(screen_width, screen_height);

  float smoothed_physics_ms = 0.0f;
  float smoothed_render_ms = 0.0f;
  float fps_title_timer = 0.0f;
  constexpr float k_fps_alpha = 0.1f;
  constexpr float k_fps_title_interval = 0.5f;
  glfwSwapInterval(0);

  while (!glfwWindowShouldClose(window)) {
    float current_frame = glfwGetTime();
    delta_time = current_frame - last_frame;
    last_frame = current_frame;

    fps_title_timer += delta_time;
    if (fps_title_timer >= k_fps_title_interval) {
      int render_fps = smoothed_render_ms > 0.0f ? static_cast<int>(1000.0f / smoothed_render_ms) : 0;
      glfwSetWindowTitle(window, ("car control demo | render-only: " + std::to_string(render_fps) + " FPS (" +
                                  std::to_string(smoothed_render_ms).substr(0, 4) +
                                  "ms) | physics: " + std::to_string(smoothed_physics_ms).substr(0, 7) + "ms")
                                     .c_str());
      fps_title_timer = 0.0f;
    }

    ProcessInput(window, camera);
    ProcessCarInput(window);
    match.SetCarInput({throttle, steering});

    double physics_start = glfwGetTime();
    match.Tick(delta_time);
    float physics_ms = static_cast<float>((glfwGetTime() - physics_start) * 1000.0);
    smoothed_physics_ms = k_fps_alpha * physics_ms + (1.0f - k_fps_alpha) * smoothed_physics_ms;

    double render_start = glfwGetTime();
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Shadow pass
    glm::mat4 light_projection = glm::ortho(-room_size, room_size, -room_size, room_size, light_projection_near_plane,
                                            light_projection_far_plane);
    glViewport(0, 0, shadow_width, shadow_height);
    glBindFramebuffer(GL_FRAMEBUFFER, depth_map_fbo);
    glClear(GL_DEPTH_BUFFER_BIT);
    glm::vec3 light_dir = glm::normalize(glm::vec3(0.0f) - light_pos);
    glm::vec3 up_vector = std::abs(light_dir.y) > 0.99f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    glm::mat4 light_space_matrix = light_projection * glm::lookAt(light_pos, glm::vec3(0.0f), up_vector);
    simple_depth_shader.Use();
    simple_depth_shader.SetMat4("lightSpaceMatrix", light_space_matrix);
    if (match.GetGround()) renderer.DrawBox(*match.GetGround(), simple_depth_shader);
    if (match.GetBall()) renderer.DrawSphere(*match.GetBall(), simple_depth_shader);
    // for (const engine::physics::Box& wall : match.GetWalls()) renderer.DrawBox(wall, simple_depth_shader);
    for (const engine::physics::Box& box : match.GetBoxes()) renderer.DrawBox(box, simple_depth_shader);
    GL_CHECK();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Main pass
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, mode->width, mode->height);
    model_loading_shader.Use();
    model_loading_shader.SetMat4("lightSpaceMatrix", light_space_matrix);
    model_loading_shader.SetInt("shadowMap", 8);
    glActiveTexture(GL_TEXTURE8);
    glBindTexture(GL_TEXTURE_2D, depth_map);
    model_loading_shader.SetBool("useTexture", false);
    model_loading_shader.SetVec3("lightColor", glm::vec3(1.0f));
    model_loading_shader.SetVec3("lightPos", light_pos);
    model_loading_shader.SetVec3("viewPos", camera.position);
    if (match.GetGround()) {
      model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.55f));
      renderer.DrawBox(*match.GetGround(), model_loading_shader, camera);
    }
    // model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.2f));
    // for (const engine::physics::Box& wall : match.GetWalls()) renderer.DrawBox(wall, model_loading_shader, camera);
    if (match.GetBall()) {
      model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.8f, 0.3f, 0.1f));
      renderer.DrawSphere(*match.GetBall(), model_loading_shader, camera);
    }
    for (const engine::physics::Box& wall : match.GetWalls())
      renderer.DrawBoxWireframe(wall, model_loading_shader, camera);
    for (const engine::physics::Box& box : match.GetBoxes()) {
      renderer.DrawBoxWireframe(box, model_loading_shader, camera);
      renderer.DrawModel(car, model_loading_shader, camera, box.position, glm::vec3(1.0f), box.rotation);
    }

    // Orientation gizmo (top-right): +X red, +Y green, +Z blue.
    renderer.DrawOrientationGizmo(camera, model_loading_shader);

    glfwSwapBuffers(window);
    float render_ms = static_cast<float>((glfwGetTime() - render_start) * 1000.0);
    smoothed_render_ms = k_fps_alpha * render_ms + (1.0f - k_fps_alpha) * smoothed_render_ms;

    glfwPollEvents();
  }

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
