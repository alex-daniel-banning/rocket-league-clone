#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <engine/render/Camera.hpp>
#include <engine/render/Renderer.hpp>

namespace engine::render
{
static constexpr float cubeVertices[] = {
    -0.5f, -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,  0.5f,  -0.5f, 0.5f,
    0.0f,  0.0f,  1.0f,  0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,

    0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  -0.5f, 0.5f,  0.5f,
    0.0f,  0.0f,  1.0f,  -0.5f, -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,

    -0.5f, -0.5f, -0.5f, 0.0f,  0.0f,  -1.0f, 0.5f,  0.5f,  -0.5f,
    0.0f,  0.0f,  -1.0f, 0.5f,  -0.5f, -0.5f, 0.0f,  0.0f,  -1.0f,

    0.5f,  0.5f,  -0.5f, 0.0f,  0.0f,  -1.0f, -0.5f, -0.5f, -0.5f,
    0.0f,  0.0f,  -1.0f, -0.5f, 0.5f,  -0.5f, 0.0f,  0.0f,  -1.0f,

    -0.5f, -0.5f, -0.5f, -1.0f, 0.0f,  0.0f,  -0.5f, -0.5f, 0.5f,
    -1.0f, 0.0f,  0.0f,  -0.5f, 0.5f,  0.5f,  -1.0f, 0.0f,  0.0f,

    -0.5f, 0.5f,  0.5f,  -1.0f, 0.0f,  0.0f,  -0.5f, 0.5f,  -0.5f,
    -1.0f, 0.0f,  0.0f,  -0.5f, -0.5f, -0.5f, -1.0f, 0.0f,  0.0f,

    0.5f,  -0.5f, -0.5f, 1.0f,  0.0f,  0.0f,  0.5f,  0.5f,  0.5f,
    1.0f,  0.0f,  0.0f,  0.5f,  -0.5f, 0.5f,  1.0f,  0.0f,  0.0f,

    0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  0.5f,  -0.5f, -0.5f,
    1.0f,  0.0f,  0.0f,  0.5f,  0.5f,  -0.5f, 1.0f,  0.0f,  0.0f,

    -0.5f, 0.5f,  -0.5f, 0.0f,  1.0f,  0.0f,  -0.5f, 0.5f,  0.5f,
    0.0f,  1.0f,  0.0f,  0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,

    0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.5f,  0.5f,  -0.5f,
    0.0f,  1.0f,  0.0f,  -0.5f, 0.5f,  -0.5f, 0.0f,  1.0f,  0.0f,

    -0.5f, -0.5f, -0.5f, 0.0f,  -1.0f, 0.0f,  0.5f,  -0.5f, 0.5f,
    0.0f,  -1.0f, 0.0f,  -0.5f, -0.5f, 0.5f,  0.0f,  -1.0f, 0.0f,

    0.5f,  -0.5f, 0.5f,  0.0f,  -1.0f, 0.0f,  -0.5f, -0.5f, -0.5f,
    0.0f,  -1.0f, 0.0f,  0.5f,  -0.5f, -0.5f, 0.0f,  -1.0f, 0.0f};

static const float cubeWireVertices[8][3] = {
    {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f},
    {-0.5f, -0.5f, 0.5f},  {0.5f, -0.5f, 0.5f},  {0.5f, 0.5f, 0.5f},  {-0.5f, 0.5f, 0.5f}};

static constexpr unsigned int cubeWireIndices[] = {0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6,
                                                   6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7};

Renderer::Renderer()
{
    initCube();
    initWireCube();
}

void Renderer::drawBox(const engine::physics::Box &box, engine::render::Shader &shader,
                       const Camera &camera)
{
    glm::mat4 model = makeModelMatrix(box);
    glm::mat4 view  = camera.GetViewMatrix();
    glm::vec3 scale(box.size.x, box.size.y, box.size.z);
    glm::mat4 projection;
    projection = getProjection(800.0f / 600.0f);

    shader.use();
    shader.setMat4("model", model);
    shader.setMat4("view", view);
    shader.setMat4("projection", projection);
    glBindVertexArray(cubeVAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
}

void Renderer::drawBoxWireframe(const engine::physics::Box &box, engine::render::Shader &shader,
                                const Camera &camera)
{
    glm::mat4 model      = makeModelMatrix(box);
    glm::mat4 view       = camera.GetViewMatrix();
    glm::mat4 projection = getProjection(800.0f / 600.0f);

    shader.use();
    shader.setMat4("model", model);
    shader.setMat4("view", view);
    shader.setMat4("projection", projection);

    glLineWidth(2.0f);
    glEnable(GL_LINE_SMOOTH);
    glBindVertexArray(cubeWireVAO);
    glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

glm::mat4 Renderer::getProjection(float aspect, float fov)
{
    return glm::perspective(glm::radians(fov), aspect, 0.1f, 100.0f);
}

glm::mat4 Renderer::makeModelMatrix(const engine::physics::Box &box)
{
    return glm::scale(glm::mat4(1.0f), glm::vec3(box.size));
}

void Renderer::initCube()
{
    glGenVertexArrays(1, &cubeVAO);
    glGenBuffers(1, &cubeVBO);

    glBindVertexArray(cubeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

    // Positions
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // Normals
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
void Renderer::initWireCube()
{
    glGenVertexArrays(1, &cubeWireVAO);
    glGenBuffers(1, &cubeWireVBO);
    glGenBuffers(1, &cubeWireEBO);

    glBindVertexArray(cubeWireVAO);

    glBindBuffer(GL_ARRAY_BUFFER, cubeWireVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeWireVertices), cubeWireVertices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeWireEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cubeWireIndices), cubeWireIndices, GL_STATIC_DRAW);

    glBindVertexArray(0);
}

} // namespace engine::render
