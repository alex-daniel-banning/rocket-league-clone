#pragma once
#include <glm/vec3.hpp>
#include <string>

class TestUtil
{
  public:
    static void ExpectVec3Near(const glm::vec3 &expected, const glm::vec3 &actual,
                               std::string msg = "", float eps = 1e-5f);
    static void AssertVec3Near(const glm::vec3 &expected, const glm::vec3 &actual,
                               std::string msg = "", float eps = 1e-5f);
};
