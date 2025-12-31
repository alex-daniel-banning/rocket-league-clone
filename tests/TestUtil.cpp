#include "TestUtil.hpp"
#include <glm/vec3.hpp>
#include <gtest/gtest.h>

void TestUtil::ExpectVec3Near(const glm::vec3 &expected, const glm::vec3 &actual, std::string msg,
                              float eps)
{
    EXPECT_NEAR(expected.x, actual.x, eps) << msg;
    EXPECT_NEAR(expected.y, actual.y, eps) << msg;
    EXPECT_NEAR(expected.z, actual.z, eps) << msg;
}

void TestUtil::AssertVec3Near(const glm::vec3 &expected, const glm::vec3 &actual, std::string msg,
                              float eps)
{
    EXPECT_NEAR(expected.x, actual.x, eps) << msg;
    EXPECT_NEAR(expected.y, actual.y, eps) << msg;
    EXPECT_NEAR(expected.z, actual.z, eps) << msg;
}
