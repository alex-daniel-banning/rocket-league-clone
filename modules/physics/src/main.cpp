#include "engine/render/Model.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <cmath>

#include <fstream>
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
void CheckGLError(const char *where);
void renderQuad();

// ugly, just getting to work for now
void tempDrawModel(engine::render::Model &model, engine::render::Shader &shader,
                   const engine::render::Camera &camera, glm::vec3 position, glm::vec3 scale,
                   glm::quat rotation);
void tempDrawModelShadow(engine::render::Model &model, engine::render::Shader &shader,
                         const engine::render::Camera &camera, glm::vec3 position, glm::vec3 scale,
                         glm::quat rotation);

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
    // GLFWwindow *window =
    //    glfwCreateWindow(mode->width, mode->height, "Fullscreen example", primary, nullptr);
    GLFWwindow *window =
        glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Non-Fullscreen example", nullptr, nullptr);

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

    engine::render::Camera camera(glm::vec3(0.0f, 7.0f, 0.01f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f,
                                  0.0f);
    camera.lookAtOrigin();
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
    glm::vec3 lightPos = glm::vec3(0.0f, 7.0f, 0.0f);
    lightingShader.use();
    lightingShader.setVec3("objectColor", 1.0f, 0.0f, 0.0f);
    lightingShader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
    lightingShader.setVec3("lightPos", lightPos);

    // Setting
    engine::render::Shader modelShader(
        engine::PathManager::moduleAsset("render-car", "shaders/model_loading.vert").c_str(),
        engine::PathManager::moduleAsset("render-car", "shaders/model_loading.frag").c_str());
    modelShader.use();
    modelShader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
    modelShader.setVec3("lightPos", lightPos);
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

    // Shadows
    unsigned int depthMapFBO;
    glGenFramebuffers(1, &depthMapFBO);

    const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
    unsigned int depthMap;
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    engine::render::Shader simpleDepthShader(
        engine::PathManager::globalAsset("shaders/simple_depth_shader.vert").c_str(),
        engine::PathManager::globalAsset("shaders/empty_shader.frag").c_str());
    engine::render::Shader debugDepthQuad(
        engine::PathManager::globalAsset("shaders/debugQuad.vert").c_str(),
        engine::PathManager::globalAsset("shaders/debugDepthQuad.frag").c_str());
    engine::render::Shader basic(engine::PathManager::globalAsset("shaders/basic.vert").c_str(),
                                 engine::PathManager::globalAsset("shaders/basic.frag").c_str());

    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float planeVertices[] = {
        // positions            // normals         // texcoords
        25.0f, -0.5f, 25.0f, 0.0f,  1.0f,   0.0f,  25.0f,  0.0f, -25.0f, -0.5f, 25.0f,  0.0f,
        1.0f,  0.0f,  0.0f,  0.0f,  -25.0f, -0.5f, -25.0f, 0.0f, 1.0f,   0.0f,  0.0f,   25.0f,

        25.0f, -0.5f, 25.0f, 0.0f,  1.0f,   0.0f,  25.0f,  0.0f, -25.0f, -0.5f, -25.0f, 0.0f,
        1.0f,  0.0f,  0.0f,  25.0f, 25.0f,  -0.5f, -25.0f, 0.0f, 1.0f,   0.0f,  25.0f,  25.0f};
    // plane VAO
    unsigned int planeVAO;
    unsigned int planeVBO;
    glGenVertexArrays(1, &planeVAO);
    glGenBuffers(1, &planeVBO);
    glBindVertexArray(planeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(6 * sizeof(float)));
    glBindVertexArray(0);
    debugDepthQuad.use();
    debugDepthQuad.setInt("depthMap", 0);

    float near_plane = 1.0f, far_plane = 25.0f;
    bool firstPass = true;
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

        /*
         */
        // Shadows
        // 1. first render to depth depthMap
        // todo, configure shader and matrices?
        glm::mat4 lightProjection = glm::ortho(-50.0f, 50.0f, -50.0f, 50.0f, near_plane, far_plane);
        glm::mat4 lightView = glm::lookAt(glm::vec3(0.0f, 9.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f),
                                          glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 lightSpaceMatrix = lightProjection * lightView;

        simpleDepthShader.use();
        simpleDepthShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            std::cerr << "Shadow FBO incomplete\n";
        }
        glClear(GL_DEPTH_BUFFER_BIT);

        ////////////////////////////todo, render scene?///////////////////////////
        tempDrawModelShadow(grass, simpleDepthShader, camera, ground.position,
                            glm::vec3(ground.size.x, 1.0f, ground.size.y), ground.rotation);
        //  tempDrawModelShadow(brick, simpleDepthShader, camera, wall_1.position,
        //                      glm::vec3(wall_1.size.x, 1.0f, wall_1.size.y), wall_1.rotation);
        //  tempDrawModelShadow(brick, simpleDepthShader, camera, wall_2.position,
        //                      glm::vec3(wall_2.size.x, 1.0f, wall_2.size.y), wall_2.rotation);
        //  tempDrawModelShadow(brick, simpleDepthShader, camera, wall_3.position,
        //                      glm::vec3(wall_3.size.x, 1.0f, wall_3.size.y), wall_3.rotation);
        //  tempDrawModelShadow(brick, simpleDepthShader, camera, wall_4.position,
        //                      glm::vec3(wall_4.size.x, 1.0f, wall_4.size.y), wall_4.rotation);
        tempDrawModelShadow(car, simpleDepthShader, camera, carPosition, glm::vec3(1.0f),
                            carRotation);
        // mega debug time
        int w = SHADOW_WIDTH;
        int h = SHADOW_HEIGHT;

        std::vector<float> buf(w * h);
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, buf.data());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        std::ofstream out("depth_dump.txt");
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                out << buf[y * w + x] << ' ';
            }
            out << '\n';
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        ////////////////////////////////////////////////////////////////////////
        // 2. then render scene as normal with shadow mapping (using depth map)
        glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glDisable(GL_DEPTH_TEST);
        debugDepthQuad.use();
        debugDepthQuad.setFloat("near_plane", near_plane);
        debugDepthQuad.setFloat("far_plane", far_plane);
        glActiveTexture(GL_TEXTURE0);
        // todo, configure shader and matrices?
        glBindTexture(GL_TEXTURE_2D, depthMap);

        /*
        basic.use();
        basic.setVec3("objectColor", glm::vec3(0.0f, 0.0f, 1.0f));
        basic.setVec3("lightColor", glm::vec3(1.0f));
        basic.setVec3("lightPos", glm::vec3(0.0f, 5.0f, 0.0f));
        basic.setVec3("viewPos", camera.Position);
        std::cout << "camera position -> " << camera.Position.x << ", " << camera.Position.y << ", "
                  << camera.Position.z << std::endl;
        std::cout << "camera front -> " << camera.Front.x << ", " << camera.Front.y << ", "
                  << camera.Front.z << std::endl;
        glm::mat4 modelMatrix = glm::mat4(1.0f);
        modelMatrix           = glm::translate(modelMatrix, glm::vec3(0.0f));
        modelMatrix           = modelMatrix * glm::mat4_cast(glm::quat(glm::vec3(0.0f)));
        modelMatrix           = glm::scale(modelMatrix, glm::vec3(1.0f));
        glm::mat4 view;
        view = camera.GetViewMatrix();
        glm::mat4 projection;
        projection = glm::perspective(camera.projection.fov, SCR_WIDTH / SCR_HEIGHT,
                                      camera.projection.nearPlane, camera.projection.farPlane);
        basic.setMat4("model", modelMatrix);
        basic.setMat4("view", view);
        basic.setMat4("projection", projection);
        */
        renderQuad();
        glEnable(GL_DEPTH_TEST);
        //  todo, render scene?

        /*

        modelShader.use();
        modelShader.setBool("useTexture", true);
        tempDrawModel(grass, modelShader, camera, ground.position,
                      glm::vec3(ground.size.x, 1.0f, ground.size.y), ground.rotation);
        tempDrawModel(grass, modelShader, camera, ground.position,
                      glm::vec3(ground.size.x, 1.0f, ground.size.y), ground.rotation);
        tempDrawModel(brick, modelShader, camera, wall_1.position,
                      glm::vec3(wall_1.size.x, 1.0f, wall_1.size.y), wall_1.rotation);
        tempDrawModel(brick, modelShader, camera, wall_2.position,
                      glm::vec3(wall_2.size.x, 1.0f, wall_2.size.y), wall_2.rotation);
        tempDrawModel(brick, modelShader, camera, wall_3.position,
                      glm::vec3(wall_3.size.x, 1.0f, wall_3.size.y), wall_3.rotation);
        tempDrawModel(brick, modelShader, camera, wall_4.position,
                      glm::vec3(wall_4.size.x, 1.0f, wall_4.size.y), wall_4.rotation);
        modelShader.setBool("useTexture", false);
        tempDrawModel(car, modelShader, camera, carPosition, glm::vec3(1.0f), carRotation);

        */

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
        if (firstPass)
        {
            CheckGLError("peepeepoopoo");
        }
        if (firstPass)
        {
            firstPass = false;
        }
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

void tempDrawModelShadow(engine::render::Model &model, engine::render::Shader &shader,
                         const engine::render::Camera &camera, glm::vec3 position, glm::vec3 scale,
                         glm::quat rotation)
{
    shader.use();
    glm::mat4 modelMatrix = glm::mat4(1.0f);
    modelMatrix           = glm::translate(modelMatrix, position);
    modelMatrix           = modelMatrix * glm::mat4_cast(rotation);
    modelMatrix           = glm::scale(modelMatrix, scale);
    shader.setMat4("model", modelMatrix);
    model.Draw(shader);
}

void tempDrawModel(engine::render::Model &model, engine::render::Shader &shader,
                   const engine::render::Camera &camera, glm::vec3 position, glm::vec3 scale,
                   glm::quat rotation)
{
    shader.use();
    glm::mat4 modelMatrix = glm::mat4(1.0f);
    modelMatrix           = glm::translate(modelMatrix, position);
    modelMatrix           = modelMatrix * glm::mat4_cast(rotation);
    modelMatrix           = glm::scale(modelMatrix, scale);
    glm::mat4 view;
    view = camera.GetViewMatrix();
    glm::mat4 projection;
    projection = glm::perspective(camera.projection.fov, SCR_WIDTH / SCR_HEIGHT,
                                  camera.projection.nearPlane, camera.projection.farPlane);
    shader.setMat4("model", modelMatrix);
    shader.setMat4("view", view);
    shader.setMat4("projection", projection);
    shader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
    shader.setVec3("lightPos", 0.0f, 7.0f, 0.0f);
    model.Draw(shader);
}

void CheckGLError(const char *where)
{
    GLenum err;
    bool hit = false;

    while ((err = glGetError()) != GL_NO_ERROR)
    {
        std::cout << "GL ERROR at " << where << ": " << err << "\n";
        hit = true;
    }

    if (hit)
        assert(false);
}

// renderQuad() renders a 1x1 XY quad in NDC
// -----------------------------------------
unsigned int quadVAO = 0;
unsigned int quadVBO;
void renderQuad()
{
    if (quadVAO == 0)
    {
        float quadVertices[] = {
            // positions        // texture Coords
            -5.0f, 0.0f, 5.0f, 0.0f, 5.0f, -5.0f, 0.0f, -5.0f, 0.0f, 0.0f,
            5.0f,  0.0f, 5.0f, 5.0f, 5.0f, 5.0f,  0.0f, -5.0f, 5.0f, 0.0f,
        };
        float quadVerticesOld[] = {
            // positions        // texture Coords
            -1.0f, 1.0f, 0.0f, 0.0f, 1.0f, -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
            1.0f,  1.0f, 0.0f, 1.0f, 1.0f, 1.0f,  -1.0f, 0.0f, 1.0f, 0.0f,
        };
        // setup plane VAO
        glGenVertexArrays(1, &quadVAO);
        glGenBuffers(1, &quadVBO);
        glBindVertexArray(quadVAO);
        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                              (void *)(3 * sizeof(float)));
    }
    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
}
