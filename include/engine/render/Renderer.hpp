#pragma once

#include <engine/physics/Box.hpp>
#include <engine/physics/Plane.hpp>
#include <engine/physics/Sphere.hpp>
#include <engine/render/Camera.hpp>
#include <engine/render/Model.hpp>
#include <engine/render/Shader.hpp>
#include <glm/gtc/quaternion.hpp>

namespace engine::render {

class Renderer {
 public:
  Renderer(const float screenWidth, const float screenHeight);
  void drawBox(const engine::physics::Box &box, Shader &shader,
               const Camera &camera);
  void drawBox(const engine::physics::Box &box, Shader &shader);
  void drawBoxWireframe(const engine::physics::Box &box, Shader &shader,
                        const Camera &camera);
  void drawSphere(const engine::physics::Sphere &sphere, Shader &shader,
                  const Camera &camera);
  void drawSphere(const engine::physics::Sphere &sphere, Shader &shader);
  void drawSphereWireframe(const engine::physics::Sphere &sphere,
                           Shader &shader, const Camera &camera);

  void drawModel(Model &model, Shader &shader, const Camera &camera,
                 glm::vec3 position, glm::vec3 scale, glm::quat rotation);
  void drawPhysicsPlane(const physics::Plane &plane, Shader &shader);
  void drawPhysicsPlane(const physics::Plane &plane, Shader &shader,
                        const Camera &camera);
  void drawPhysicsPlaneNormal(const physics::Plane &plane, Shader &shader,
                              const Camera &camera);

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
  unsigned int lineVAO, lineVBO;
  void initCube();
  void initWireCube();
  void initSphere();
  void initPlane();
  void initLine();
};

}  // namespace engine::render
