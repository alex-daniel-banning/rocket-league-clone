#include <glm/gtc/quaternion.hpp>

namespace engine {

class Math {
 public:
  static glm::quat GetRotationFromEulerAngles(glm::vec3 euler) {
    return glm::quat(glm::vec3(glm::radians(euler.x), glm::radians(euler.y),
                               glm::radians(euler.z)));
  }
};

}  // namespace engine
