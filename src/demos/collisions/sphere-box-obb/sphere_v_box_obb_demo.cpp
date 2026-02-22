#define GLM_ENABLE_EXPERIMENTAL
#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <engine/match.hpp>
#include <engine/path_manager.hpp>
#include <engine/physics/box_builder.hpp>
#include <engine/physics/sphere.hpp>
#include <engine/render/renderer.hpp>
#include <engine/render/shader.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

#include "../../demo_common.hpp"

#define GL_CHECK()                                                                                               \
  do {                                                                                                           \
    GLenum err;                                                                                                  \
    while ((err = glGetError()) != GL_NO_ERROR) {                                                                \
      std::cerr << "OpenGL error at " << __FILE__ << ":" << __LINE__ << " -> 0x" << std::hex << err << std::dec \
                << '\n';                                                                                         \
    }                                                                                                            \
  } while (0)

int main() {
  float ground_size = 50.0f;
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
          .WithWalls(walls)
          .WithBox(engine::physics::Box(glm::vec3(5.0f), glm::vec3(0.0f, 3.0f, 0.0f), glm::vec3(0.0f), 30.0f,
                                        glm::quat(), glm::vec3(0.0f, 0.0f, 0.0f)))
          .Build();

  auto [window, mode, screen_width, screen_height] = InitWindow("sphere v box OBB demo");
  glEnable(GL_DEPTH_TEST);

  engine::render::Camera camera(glm::vec3(24.0f, 10.0f, 24.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f);
  camera.LookAt(glm::vec3(0.0f, 5.0f, 0.0f));
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

  float demo_start = glfwGetTime();
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

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Shadow pass
    glm::mat4 light_projection =
        glm::ortho(-ground_size, ground_size, -ground_size, ground_size, light_projection_near_plane,
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
      model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.2f));
      renderer.DrawBox(*match.GetGround(), model_loading_shader, camera);
    }
    model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.3f));
    for (const engine::physics::Box& wall : match.GetWalls()) renderer.DrawBox(wall, model_loading_shader, camera);
    if (match.GetBall()) {
      model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.0f, 0.5f, 0.3f));
      renderer.DrawSphere(*match.GetBall(), model_loading_shader, camera);
    }
    if (!match.GetBoxes().empty()) model_loading_shader.SetVec3("diffuseColor", glm::vec3(0.0f, 0.5f, 0.3f));
    for (const engine::physics::Box& box : match.GetBoxes()) renderer.DrawBox(box, model_loading_shader, camera);

    // Wireframes
    for (const engine::physics::Box& wall : match.GetWalls()) renderer.DrawBoxWireframe(wall, model_loading_shader, camera);
    if (match.GetGround()) renderer.DrawBoxWireframe(*match.GetGround(), model_loading_shader, camera);
    for (const engine::physics::Box& box : match.GetBoxes()) renderer.DrawBoxWireframe(box, model_loading_shader, camera);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
