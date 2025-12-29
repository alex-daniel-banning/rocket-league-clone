#pragma once
#include <glm/vec3.hpp>

class TestUtil
{
  public:
    static void ExpectVec3Near(const glm::vec3 &a, const glm::vec3 &b, float eps = 1e-5f);
    static void AssertVec3Near(const glm::vec3 &a, const glm::vec3 &b, float eps = 1e-5f);
};
