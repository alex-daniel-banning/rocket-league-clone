#include "engine/render/Model.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <cmath>

#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/dual_quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

#include <engine/PathManager.hpp>
#include <engine/physics/Box.hpp>
#include <engine/physics/Plane.hpp>
#include <engine/render/Camera.hpp>
#include <engine/render/Renderer.hpp>
#include <engine/render/Shader.hpp>

float SCR_WIDTH  = 800.0f;
float SCR_HEIGHT = 600.0f;

unsigned int loadTexture(const char *path);

float deltaTime = 0.0f; // Time between current frame and last frame
float lastFrame = 0.0f; // Time of last frame

const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
const float LIGHT_PROJECTION_NEAR_PLANE = 1.0f, LIGHT_PROJECTION_FAR_PLANE = 50.0f;
unsigned int depthMapFBO;
unsigned int depthMap;
void setupShadowBuffer();

int main()
{
    // Initialize GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    // Tell GLFW what kind of context to create
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWmonitor *primary    = glfwGetPrimaryMonitor();
    const GLFWvidmode *mode = glfwGetVideoMode(primary);

    GLFWwindow *window =
        glfwCreateWindow(mode->width, mode->height, "Fullscreen Window", primary, NULL);

    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);
    // glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Initialize glad
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }

    glEnable(GL_DEPTH_TEST);

    setupShadowBuffer();

    glm::vec3 lightPos(1.0f, 40.0f, 1.0f);
    engine::render::Shader simpleDepthShader(
        engine::PathManager::globalAsset("shaders/simple_depth_shader.vert").c_str(),
        engine::PathManager::globalAsset("shaders/empty_shader.frag").c_str());

    engine::render::Camera camera(glm::vec3(3.0f, 7.0f, 7.01f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f,
                                  0.0f);
    camera.lookAtOrigin();
    camera.projection.farPlane = 300.0f;

    glm::quat carRotation = glm::quat(glm::vec3(0.0f));
    engine::render::Model car(
        engine::PathManager::moduleAsset("render-car", "car/car.obj").c_str());
    glm::vec3 carPosition = glm::vec3(0.0f, 2.0f, 0.0f);
    camera.lookAt(carPosition);

    // create regular shader
    engine::render::Shader modelLoadingShader(
        engine::PathManager::globalAsset("shaders/model_loading.vert").c_str(),
        engine::PathManager::globalAsset("shaders/model_loading.frag").c_str());

    // Add quad/floor for shadows to appear on
    engine::render::Model floor(engine::PathManager::globalAsset("floor/floor.obj").c_str());
    engine::physics::Plane ground;
    float pitchLength = 150.0f;
    float pitchWidth  = 75.0f;
    ground.size       = glm::vec2(1, 1);
    ground.position   = glm::vec3(0.0f, 0.0f, 0.0f);
    ground.rotation   = glm::angleAxis(glm::radians(0.0f), glm::vec3(1, 0, 0));

    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = glfwGetTime();
        deltaTime          = currentFrame - lastFrame;
        lastFrame          = currentFrame;

        // render
        // ------
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 1. render depth of scene to texture (from light's perspective)
        // --------------------------------------------------------------
        glm::mat4 lightProjection, lightView;
        glm::mat4 lightSpaceMatrix;
        lightProjection = glm::ortho(-20.0f, 20.0f, -20.0f, 20.0f, LIGHT_PROJECTION_NEAR_PLANE,
                                     LIGHT_PROJECTION_FAR_PLANE);
        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glClear(GL_DEPTH_BUFFER_BIT);
        lightView        = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0, 1.0, 0.0));
        lightSpaceMatrix = lightProjection * lightView;
        simpleDepthShader.use();
        simpleDepthShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

        simpleDepthShader.use();
        glm::mat4 modelMatrix = glm::mat4(1.0f);
        modelMatrix           = glm::translate(modelMatrix, carPosition);
        modelMatrix           = modelMatrix * glm::mat4_cast(carRotation);
        modelMatrix           = glm::scale(modelMatrix, glm::vec3(1.0f));
        simpleDepthShader.setMat4("model", modelMatrix);
        car.Draw(simpleDepthShader);
        modelMatrix = glm::mat4(1.0f);
        modelMatrix = glm::translate(modelMatrix, ground.position);
        modelMatrix = modelMatrix * glm::mat4_cast(ground.rotation);
        modelMatrix = glm::scale(modelMatrix, glm::vec3(ground.size.x, 1.0f, ground.size.y));
        simpleDepthShader.setMat4("model", modelMatrix);
        floor.Draw(simpleDepthShader);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // reset viewport
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glViewport(0, 0, mode->width, mode->height);

        //  Render Car Model
        modelLoadingShader.use();
        modelMatrix = glm::mat4(1.0f);
        modelMatrix = glm::translate(modelMatrix, carPosition);
        modelMatrix = modelMatrix * glm::mat4_cast(carRotation);
        modelMatrix = glm::scale(modelMatrix, glm::vec3(1.0f));
        glm::mat4 view;
        view = camera.GetViewMatrix();
        glm::mat4 projection;
        projection = glm::perspective(camera.projection.fov, SCR_WIDTH / SCR_HEIGHT,
                                      camera.projection.nearPlane, camera.projection.farPlane);
        modelLoadingShader.setMat4("model", modelMatrix);
        modelLoadingShader.setMat4("view", view);
        modelLoadingShader.setMat4("projection", projection);
        modelLoadingShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);
        modelLoadingShader.setInt("shadowMap", 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        modelLoadingShader.setBool("useTexture", false);
        modelLoadingShader.setVec3("diffuseColor", glm::vec3(0.0f, 0.0f, 0.7f));
        modelLoadingShader.setVec3("lightColor", glm::vec3(1.0f));
        modelLoadingShader.setVec3("lightPos", lightPos);
        modelLoadingShader.setVec3("viewPos", camera.Position);
        car.Draw(modelLoadingShader);

        // Render Floor Model
        modelMatrix = glm::mat4(1.0f);
        modelMatrix = glm::translate(modelMatrix, ground.position);
        modelMatrix = modelMatrix * glm::mat4_cast(ground.rotation);
        modelMatrix = glm::scale(modelMatrix, glm::vec3(ground.size.x, 1.0f, ground.size.y));
        modelLoadingShader.setMat4("model", modelMatrix);
        modelLoadingShader.setMat4("view", view);
        modelLoadingShader.setMat4("projection", projection);
        modelLoadingShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);
        modelLoadingShader.setInt("shadowMap", 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        modelLoadingShader.setBool("useTexture", false);
        modelLoadingShader.setVec3("diffuseColor", glm::vec3(1.0f, 1.0f, 1.0f));
        modelLoadingShader.setVec3("lightColor", glm::vec3(1.0f));
        modelLoadingShader.setVec3("lightPos", lightPos);
        modelLoadingShader.setVec3("viewPos", camera.Position);
        floor.Draw(modelLoadingShader);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Clean up
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

void setupShadowBuffer()
{
    glGenFramebuffers(1, &depthMapFBO);
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
