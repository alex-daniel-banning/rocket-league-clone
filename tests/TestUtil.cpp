#include "TestUtil.hpp"
#include <glm/vec3.hpp>
#include <gtest/gtest.h>

void TestUtil::ExpectVec3Near(const glm::vec3 &expected, const glm::vec3 &actual, float eps)
{
    EXPECT_NEAR(expected.x, actual.x, eps);
    EXPECT_NEAR(expected.y, actual.y, eps);
    EXPECT_NEAR(expected.z, actual.z, eps);
}

void TestUtil::AssertVec3Near(const glm::vec3 &expected, const glm::vec3 &actual, float eps)
{
    EXPECT_NEAR(expected.x, actual.x, eps);
    EXPECT_NEAR(expected.y, actual.y, eps);
    EXPECT_NEAR(expected.z, actual.z, eps);
}
