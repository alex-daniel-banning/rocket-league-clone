#pragma once
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <iostream>

class Print {
 public:
  // Print a glm::vec3 to std::cout
  static void vec3(const glm::vec3& v, const std::string& label = "") {
    if (!label.empty()) {
      std::cout << label << ": ";
    }
    std::cout << "(" << v.x << ", " << v.y << ", " << v.z << ")\n";
  }
  // Print a glm::quat as w + xi + yj + zk
  static void quat(const glm::quat& q, const std::string& label = "") {
    if (!label.empty()) std::cout << label << ": ";
    std::cout << "(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z
              << ")\n";
  }

  // Print as Euler angles (degrees)
  static void quatEuler(const glm::quat& q, const std::string& label = "") {
    glm::vec3 euler = glm::eulerAngles(q);  // radians
    euler = glm::degrees(euler);
    if (!label.empty()) std::cout << label << ": ";
    std::cout << "Euler(deg) = (" << euler.x << ", " << euler.y << ", "
              << euler.z << ")\n";
  }
};

// Optional: overload << for glm::vec3 for convenience
inline std::ostream& operator<<(std::ostream& os, const glm::vec3& v) {
  os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return os;
}
