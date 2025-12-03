#include <glad/glad.h>
#include <engine/render/Renderer.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace engine::render
{

Renderer::Renderer()
{
    // todo, see if there's a way for clang-format to not affect this (so I can label with comments)
    float vertices[] = {-0.5f, -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,  0.5f,  -0.5f, 0.5f,
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

    glGenVertexArrays(1, &cubeVAO);
    glGenBuffers(1, &cubeVBO);

    glBindVertexArray(cubeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Positions
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // Normals
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Renderer::drawBox(const engine::physics::Box &box, engine::render::Shader &shader)
{
    glm::mat4 model = glm::mat4(1.0f);
    glm::vec3 scale(box.size.x, box.size.y, box.size.z);
    model = glm::scale(model, scale);

    shader.use();
    shader.setMat4("model", model);
    glBindVertexArray(cubeVAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
}
} // namespace engine::render
