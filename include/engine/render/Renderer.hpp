#pragma once

#include <glm/gtc/quaternion.hpp>

#include <engine/physics/Box.hpp>
#include <engine/physics/Plane.hpp>
#include <engine/physics/Sphere.hpp>
#include <engine/render/Camera.hpp>
#include <engine/render/Model.hpp>
#include <engine/render/Shader.hpp>

namespace engine::render
{

class Renderer
{
  public:
    Renderer(const float screenWidth, const float screenHeight);
    void drawBox(const engine::physics::Box &box, engine::render::Shader &shader,
                 const Camera &camera);
    void drawBoxWireframe(const engine::physics::Box &box, engine::render::Shader &shader,
                          const Camera &camera);
    void drawSphere(const engine::physics::Sphere &sphere, engine::render::Shader &shader,
                    const Camera &camera);
    void drawSphereWireframe(const engine::physics::Sphere &sphere, engine::render::Shader &shader,
                             const Camera &camera);

    void drawModel(engine::render::Model &model, engine::render::Shader &shader,
                   const Camera &camera, glm::vec3 position, glm::vec3 scale, glm::quat rotation);
    void drawPhysicsPlane(const physics::Plane &plane, Shader &shader);
    void drawPhysicsPlane(const physics::Plane &plane, Shader &shader, const Camera &camera);

    glm::mat4 makeModelMatrix(const engine::physics::Box &box);
    glm::mat4 makeModelMatrix(const engine::physics::Sphere &sphere);

    // todo, I don't know what these are useful for, but chat gpt suggested
    // void beginFrame();
    // void endFrame();

  private:
    float screenWidth, screenHeight;
    float nearPlane, farPlane;
    unsigned int cubeVAO, cubeVBO;
    unsigned int cubeWireVAO, cubeWireVBO, cubeWireEBO;
    unsigned int sphereVAO, sphereEBO, sphereIndexCount;
    unsigned int planeVAO, planeVBO;
    void initCube();
    void initWireCube();
    void initSphere();
    void initPlane();
};

} // namespace engine::render
