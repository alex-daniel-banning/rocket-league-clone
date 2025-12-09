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

float deltaTime = 0.0f; // Time between current frame and last frame
float lastFrame = 0.0f; // Time of last frame

float SCR_WIDTH  = 800.0f;
float SCR_HEIGHT = 600.0f;
float lastX      = SCR_WIDTH / 2.0f;
float lastY      = SCR_HEIGHT / 2.0f;
bool firstMouse  = true;

void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void processInput(GLFWwindow *window, engine::render::Camera &camera);

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

    // Create a windowed mode window
    GLFWwindow *window =
        glfwCreateWindow(mode->width, mode->height, "Fullscreen example", primary, nullptr);

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

    glEnable(GL_DEPTH_TEST);

    engine::render::Camera camera(glm::vec3(0.0f, 5.0f, 11.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f,
                                  0.0f);
    camera.projection.farPlane = 300.0f;
    camera.MovementSpeed       = 10.0f;
    glfwSetWindowUserPointer(window, &camera);

    engine::render::Renderer renderer(SCR_WIDTH, SCR_HEIGHT);
    engine::physics::Box box;
    box.size.x = 1.0f;
    box.size.y = 2.0f;
    box.size.z = 4.0f;

    engine::physics::Sphere sphere;
    sphere.radius   = 1.0f;
    sphere.position = glm::vec3(0.0f, 0.0f, 0.0f);
    sphere.velocity = glm::vec3(1.0f, 0.0f, 0.0f);

    engine::render::Shader lightingShader(
        engine::PathManager::globalAsset("shaders/basic.vert").c_str(),
        engine::PathManager::globalAsset("shaders/basic.frag").c_str());
    lightingShader.use();
    lightingShader.setVec3("objectColor", 1.0f, 0.0f, 0.0f);
    lightingShader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
    lightingShader.setVec3("lightPos", 0.0f, 5.0f, 0.0f);

    // Setting
    engine::render::Shader modelShader(
        engine::PathManager::moduleAsset("render-car", "shaders/model_loading.vert").c_str(),
        engine::PathManager::moduleAsset("render-car", "shaders/model_loading.frag").c_str());
    modelShader.use();
    modelShader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
    modelShader.setVec3("lightPos", 0.0f, 5.0f, 0.0f);
    modelShader.setFloat("textureRepeat", 5.0f);
    float pitchLength = 150.0f;
    float pitchWidth  = 75.0f;
    float pitchHeight = 30.0f;
    // Ground
    engine::physics::Plane ground;
    ground.size     = glm::vec2(pitchLength, pitchWidth);
    ground.position = glm::vec3(0.0f, 0.0f, 0.0f);
    ground.rotation = glm::angleAxis(glm::radians(0.0f), glm::vec3(1, 0, 0));
    engine::render::Model grass(engine::PathManager::globalAsset("grass/grass.obj").c_str());
    // Walls
    glm::vec3 pitchCenter(0.0f, pitchHeight / 2.0f, 0.0f);
    engine::render::Model brick(engine::PathManager::globalAsset("brick/brick.obj").c_str());
    engine::physics::Plane wall_1;
    wall_1.size     = glm::vec2(pitchLength, pitchHeight);
    wall_1.position = glm::vec3(0, pitchHeight / 2, pitchWidth / 2);
    wall_1.rotation =
        glm::quat(glm::vec3(glm::radians(-90.0f), glm::radians(0.0f), glm::radians(0.0f)));
    engine::physics::Plane wall_2;
    wall_2.size     = glm::vec2(pitchLength, pitchHeight);
    wall_2.position = glm::vec3(0, pitchHeight / 2, -pitchWidth / 2);
    wall_2.rotation =
        glm::quat(glm::vec3(glm::radians(90.0f), glm::radians(0.0f), glm::radians(0.0f)));
    engine::physics::Plane wall_3;
    wall_3.size     = glm::vec2(pitchWidth, pitchHeight);
    wall_3.position = glm::vec3(pitchLength / 2, pitchHeight / 2, 0);
    wall_3.rotation =
        glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(90.0f), glm::radians(90.0f)));
    engine::physics::Plane wall_4;
    wall_4.size     = glm::vec2(pitchWidth, pitchHeight);
    wall_4.position = glm::vec3(-pitchLength / 2, pitchHeight / 2, 0);
    wall_4.rotation =
        glm::quat(glm::vec3(glm::radians(0.0f), glm::radians(-90.0f), glm::radians(-90.0f)));

    glm::quat carRotation = glm::quat(glm::vec3(0.0f));
    //     glm::quat(glm::vec3(glm::radians(90.0f), glm::radians(0.0f), glm::radians(90.0f)));
    engine::render::Model car(
        engine::PathManager::moduleAsset("render-car", "car/car.obj").c_str());
    glm::vec3 carPosition = glm::vec3(0.0f, 2.0f, 0.0f);
    camera.lookAt(carPosition);

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = glfwGetTime();
        deltaTime          = currentFrame - lastFrame;
        lastFrame          = currentFrame;

        processInput(window, camera);

        // Clear the screen
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        modelShader.setBool("useTexture", true);
        renderer.drawModel(grass, modelShader, camera, ground.position,
                           glm::vec3(ground.size.x, 1.0f, ground.size.y), ground.rotation);
        renderer.drawModel(brick, modelShader, camera, wall_1.position,
                           glm::vec3(wall_1.size.x, 1.0f, wall_1.size.y), wall_1.rotation);
        renderer.drawModel(brick, modelShader, camera, wall_2.position,
                           glm::vec3(wall_2.size.x, 1.0f, wall_2.size.y), wall_2.rotation);
        renderer.drawModel(brick, modelShader, camera, wall_3.position,
                           glm::vec3(wall_3.size.x, 1.0f, wall_3.size.y), wall_3.rotation);
        renderer.drawModel(brick, modelShader, camera, wall_4.position,
                           glm::vec3(wall_4.size.x, 1.0f, wall_4.size.y), wall_4.rotation);
        modelShader.setBool("useTexture", false);
        renderer.drawModel(car, modelShader, camera, carPosition, glm::vec3(1.0f), carRotation);

        GLenum err;
        while ((err = glGetError()) != GL_NO_ERROR)
        {
            std::cout << "GL ERROR: " << err << "\n";
        }

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    // Clean up
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
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
