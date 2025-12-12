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
Renderer::Renderer(const float screenWidth, const float screenHeight)
{
    this->screenWidth  = screenWidth;
    this->screenHeight = screenHeight;
    initCube();
    initWireCube();
    initSphere();
}

void Renderer::drawBox(const engine::physics::Box &box, engine::render::Shader &shader,
                       const Camera &camera)
{
    glm::mat4 model = makeModelMatrix(box);
    glm::mat4 view  = camera.GetViewMatrix();
    glm::mat4 projection;
    projection = getProjection(screenWidth / screenHeight);

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
    glm::mat4 projection = getProjection(screenWidth / screenHeight);

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

void Renderer::drawSphere(const engine::physics::Sphere &sphere, engine::render::Shader &shader,
                          const Camera &camera)
{
    glm::mat4 model = makeModelMatrix(sphere);
    glm::mat4 view  = camera.GetViewMatrix();
    glm::mat4 projection;
    projection = getProjection(screenWidth / screenHeight);

    shader.use();
    shader.setMat4("model", model);
    shader.setMat4("view", view);
    shader.setMat4("projection", projection);
    glBindVertexArray(sphereVAO);
    glDrawElements(GL_TRIANGLES, sphereIndexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Renderer::drawSphereWireframe(const engine::physics::Sphere &sphere,
                                   engine::render::Shader &shader, const Camera &camera)
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    drawSphere(sphere, shader, camera);
}

// todo, remove this, too coupled perhaps
void Renderer::drawModel(engine::render::Model &model, engine::render::Shader &shader,
                         const Camera &camera, glm::vec3 position, glm::vec3 scale,
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
    projection = glm::perspective(camera.projection.fov, screenWidth / screenHeight,
                                  camera.projection.nearPlane, camera.projection.farPlane);
    shader.setMat4("model", modelMatrix);
    shader.setMat4("view", view);
    shader.setMat4("projection", projection);
    shader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
    shader.setVec3("lightPos", 5.0f, 5.0f, 5.0f);
    model.Draw(shader);
}

void Renderer::drawModel(engine::render::Model &model, engine::render::Shader &shader)
{
    shader.use();
    model.Draw(shader);
}

glm::mat4 Renderer::getProjection(float aspect, float fov)
{
    return glm::perspective(glm::radians(fov), aspect, 0.1f, 100.0f);
}

glm::mat4 Renderer::makeModelMatrix(const engine::physics::Box &box)
{
    return glm::scale(glm::mat4(1.0f), glm::vec3(box.size));
}

glm::mat4 Renderer::makeModelMatrix(const engine::physics::Sphere &sphere)
{
    glm::mat4 model = glm::mat4(1.0f);
    model           = glm::translate(model, glm::vec3(sphere.position));
    model           = glm::scale(model, glm::vec3(sphere.radius));
    return model;
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

void Renderer::initSphere()
{

    struct Vertex
    {
        glm::vec3 position;
        glm::vec3 normal;
    };

    unsigned int stacks = 16;
    unsigned int slices = 16;
    float radius        = 0.5f;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    // top Vertex
    vertices.push_back({glm::vec3(0, radius, 0), glm::vec3(0, 1, 0)});
    for (int i = 1; i < stacks; ++i)
    {
        float V   = i / (float)stacks;
        float phi = V * glm::pi<float>();

        for (int j = 0; j <= slices; ++j)
        {
            float U     = j / (float)slices;
            float theta = U * glm::two_pi<float>();

            float x = cos(theta) * sin(phi) * radius;
            float y = cos(phi) * radius;
            float z = sin(theta) * sin(phi) * radius;

            glm::vec3 pos  = glm::vec3(x, y, z);
            glm::vec3 norm = glm::normalize(pos); // normal

            vertices.push_back({pos, norm});
        }
    }
    // bottom Vertex
    vertices.push_back({glm::vec3(0, -radius, 0), glm::vec3(0, -1, 0)});

    // Generate indices
    // top
    unsigned int topIndex  = 0;
    unsigned int ringStart = 1;
    for (int j = 0; j < slices; ++j)
    {
        indices.push_back(topIndex);
        indices.push_back(ringStart + j);
        indices.push_back(ringStart + j + 1);
    }
    // middle
    unsigned int ringVertexCount = slices + 1;

    for (unsigned int i = 0; i < stacks - 2; ++i)
    {
        unsigned int curr = 1 + i * ringVertexCount;
        unsigned int next = curr + ringVertexCount;

        for (unsigned int j = 0; j < slices; ++j)
        {
            unsigned int v0  = curr + j;
            unsigned int v1  = next + j;
            unsigned int v0n = v0 + 1;
            unsigned int v1n = v1 + 1;

            // two triangles per quad
            indices.push_back(v0);
            indices.push_back(v1);
            indices.push_back(v0n);

            indices.push_back(v1);
            indices.push_back(v1n);
            indices.push_back(v0n);
        }
    }
    // bottom
    unsigned int bottomIndex   = vertices.size() - 1;
    unsigned int lastRingStart = bottomIndex - ringVertexCount;
    for (int j = 0; j < slices; ++j)
    {
        indices.push_back(bottomIndex);
        indices.push_back(lastRingStart + j + 1);
        indices.push_back(lastRingStart + j);
    }

    // Create VAO, VBO, and EBO
    unsigned int VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    // Vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(),
                 GL_STATIC_DRAW);

    // Element buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(),
                 GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
    glEnableVertexAttribArray(0);

    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    // Store VAO/EBO count somewhere in Renderer for rendering
    this->sphereVAO        = VAO;
    this->sphereEBO        = EBO;
    this->sphereIndexCount = static_cast<unsigned int>(indices.size());
}

} // namespace engine::render
