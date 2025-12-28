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

#include <engine/Match.hpp>
#include <engine/PathManager.hpp>
#include <engine/physics/Box.hpp>
#include <engine/physics/Plane.hpp>
#include <engine/render/Camera.hpp>
#include <engine/render/Renderer.hpp>
#include <engine/render/Shader.hpp>

float SCR_WIDTH  = 800.0f;
float SCR_HEIGHT = 600.0f;

#define GL_CHECK()                                                                                 \
    do                                                                                             \
    {                                                                                              \
        GLenum err;                                                                                \
        while ((err = glGetError()) != GL_NO_ERROR)                                                \
        {                                                                                          \
            std::cerr << "OpenGL error at " << __FILE__ << ":" << __LINE__ << " -> 0x" << std::hex \
                      << err << std::dec << '\n';                                                  \
        }                                                                                          \
    } while (0)

unsigned int loadTexture(const char *path);

float lastX, lastY;
bool firstMouse = true;
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void processInput(GLFWwindow *window, engine::render::Camera &camera);

float deltaTime = 0.0f; // Time between current frame and last frame
float lastFrame = 0.0f; // Time of last frame

const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
const float LIGHT_PROJECTION_NEAR_PLANE = 1.0f, LIGHT_PROJECTION_FAR_PLANE = 70.0f;
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
    float screenWidth       = mode->width;
    float screenHeight      = mode->height;
    lastX                   = screenWidth / 2.0f;
    lastY                   = screenHeight / 2.0f;

    GLFWwindow *window =
        glfwCreateWindow(screenWidth, screenHeight, "Fullscreen Window", primary, NULL);

    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Initialize glad
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }
    // Setup completed.

    glEnable(GL_DEPTH_TEST);
    engine::render::Camera camera(glm::vec3(3.0f, 7.0f, 7.01f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f,
                                  0.0f);
    camera.lookAtOrigin();
    camera.projection.farPlane = 300.0f;
    glfwSetWindowUserPointer(window, &camera);

    setupShadowBuffer();

    glm::vec3 lightPos(0.0f, 40.0f, 0.0f);
    engine::render::Shader simpleDepthShader(
        engine::PathManager::globalAsset("shaders/simple_depth_shader.vert").c_str(),
        engine::PathManager::globalAsset("shaders/empty_shader.frag").c_str());

    // create regular shader
    engine::render::Shader modelLoadingShader(
        engine::PathManager::globalAsset("shaders/model_loading.vert").c_str(),
        engine::PathManager::globalAsset("shaders/model_loading.frag").c_str());

    engine::render::Renderer renderer(screenWidth, screenHeight);

    // todo, generate the model data dynamically/programmatically for simple objects like a plane
    glm::vec3 color(0.2f, 0.2f, 0.2f);
    float groundSize = 50.0f;
    engine::physics::Plane ground(groundSize, groundSize, color);

    glm::vec3 wallColor(0.2f, 0.15f, 0.15f);
    // clang-format off
    // Why does it not matter that my rotation is 90 or -90? i.e. I can normal isn't behaving as expected.
    float wallTranslationSize = groundSize / 2;
    std::vector<engine::physics::Plane> walls = {
        engine::physics::Plane(groundSize, groundSize, wallColor, glm::vec3(wallTranslationSize, wallTranslationSize, 0.0f), glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f))),
        engine::physics::Plane(groundSize, groundSize, wallColor, glm::vec3(-wallTranslationSize, wallTranslationSize, 0.0f), glm::angleAxis(glm::radians(-90.0f), glm::vec3(0.0f, 0.0f, 1.0f))),
        engine::physics::Plane(groundSize, groundSize, wallColor, glm::vec3(0.0f, wallTranslationSize, wallTranslationSize), glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f))),
        engine::physics::Plane(groundSize, groundSize, wallColor, glm::vec3(0.0f, wallTranslationSize, -wallTranslationSize), glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)))
    };
    // clang-format on

    engine::physics::Sphere ball;
    ball.radius   = 1.0f;
    ball.position = glm::vec3(0.0f, 2.0f, 1.0f);
    ball.velocity = glm::vec3(10.0f, 0.0f, 0.0f);

    engine::Match match(ball, ground, walls);

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = glfwGetTime();
        deltaTime          = currentFrame - lastFrame;
        lastFrame          = currentFrame;

        processInput(window, camera);

        match.tick(deltaTime);

        // render
        // ------
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 1. render depth of scene to texture (from light's perspective)
        // --------------------------------------------------------------
        glm::mat4 lightProjection, lightView;
        glm::mat4 lightSpaceMatrix;
        lightProjection = glm::ortho(-groundSize, groundSize, -groundSize, groundSize,
                                     LIGHT_PROJECTION_NEAR_PLANE, LIGHT_PROJECTION_FAR_PLANE);
        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glClear(GL_DEPTH_BUFFER_BIT);

        glm::vec3 lightDir = glm::normalize(glm::vec3(0.0f) - lightPos);
        glm::vec3 upVector = glm::vec3(0.0f, 1.0f, 0.0f);
        // If light is pointing nearly straight up or down, use a different vector
        if (std::abs(lightDir.y) > 0.99f)
        {
            upVector = glm::vec3(0.0f, 0.0f, 1.0f);
        }
        lightView = glm::lookAt(lightPos, glm::vec3(0.0f), upVector);

        lightSpaceMatrix = lightProjection * lightView;
        simpleDepthShader.use();
        simpleDepthShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

        simpleDepthShader.use();
        glm::mat4 modelMatrix = glm::mat4(1.0f);
        modelMatrix           = glm::translate(modelMatrix, ground.getPosition());
        modelMatrix           = modelMatrix * glm::mat4_cast(ground.getRotation());
        modelMatrix           = glm::scale(modelMatrix, glm::vec3(1.0f));
        simpleDepthShader.setMat4("model", modelMatrix);
        renderer.drawPhysicsPlane(match.getGround(), simpleDepthShader);

        renderer.drawSphere(match.getBall(), simpleDepthShader, camera);

        GL_CHECK();

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // reset viewport
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glViewport(0, 0, mode->width, mode->height);

        // Render Floor Model
        modelLoadingShader.use();
        modelLoadingShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);
        modelLoadingShader.setInt("shadowMap", 8);
        glActiveTexture(GL_TEXTURE8); // todo set to 8 so it doesn't conflict with model texture
                                      // index, need a better long term solution
        glBindTexture(GL_TEXTURE_2D, depthMap);
        modelLoadingShader.setBool("useTexture", false);
        modelLoadingShader.setVec3("lightColor", glm::vec3(1.0f));
        modelLoadingShader.setVec3("lightPos", lightPos);
        modelLoadingShader.setVec3("viewPos", camera.Position);
        renderer.drawPhysicsPlane(match.getGround(), modelLoadingShader, camera);
        for (engine::physics::Plane wall : match.getWalls())
        {
            renderer.drawPhysicsPlane(wall, modelLoadingShader, camera);
        }

        modelLoadingShader.setVec3("diffuseColor", glm::vec3(0.0f, 0.5f, 0.3f));
        renderer.drawSphere(match.getBall(), modelLoadingShader, camera);

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

void processInput(GLFWwindow *window, engine::render::Camera &camera)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    const float cameraSpeed = camera.MovementSpeed * deltaTime;
    glm::vec3 forward       = glm::normalize(glm::vec3(camera.Front.x, 0.0f, camera.Front.z));

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.Position += cameraSpeed * forward;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.Position -= cameraSpeed * forward;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.Position -= glm::normalize(glm::cross(camera.Front, camera.Up)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.Position += glm::normalize(glm::cross(camera.Front, camera.Up)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        camera.Position += glm::vec3(0.0f, 1.0f, 0.0f) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        camera.Position -= glm::vec3(0.0f, 1.0f, 0.0f) * cameraSpeed;

    // make sure that when camera.Pitch is out of bounds, screen doesn't get flipped
    if (camera.Pitch > 89.0f)
        camera.Pitch = 89.0f;
    if (camera.Pitch < -89.0f)
        camera.Pitch = -89.0f;

    glm::vec3 front;
    front.x      = cos(glm::radians(camera.Yaw)) * cos(glm::radians(camera.Pitch));
    front.y      = sin(glm::radians(camera.Pitch));
    front.z      = sin(glm::radians(camera.Yaw)) * cos(glm::radians(camera.Pitch));
    camera.Front = glm::normalize(front);
}

void mouse_callback(GLFWwindow *window, double xposIn, double yposIn)
{
    auto *camera = static_cast<engine::render::Camera *>(glfwGetWindowUserPointer(window));
    float mouseSensitivity = 0.05f;
    float xpos             = static_cast<float>(xposIn);
    float ypos             = static_cast<float>(yposIn);
    if (firstMouse)
    {
        lastX      = xpos;
        lastY      = ypos;
        firstMouse = false;
    }
    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    assert(camera && "Camera pointer not set via glfwSetWindowUserPointer!");
    camera->ProcessMouseMovement(xoffset, yoffset, mouseSensitivity);
}
