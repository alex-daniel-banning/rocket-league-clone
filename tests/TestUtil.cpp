#include "TestUtil.hpp"
#include <glm/vec3.hpp>
#include <gtest/gtest.h>

void TestUtil::ExpectVec3Near(const glm::vec3 &expected, const glm::vec3 &actual, std::string msg,
                              float eps)
{
    auto formatVec = [](const glm::vec3 &v)
    {
        std::ostringstream oss;
        oss << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return oss.str();
    };

    bool pass = true;
    pass &= std::abs(expected.x - actual.x) <= eps;
    pass &= std::abs(expected.y - actual.y) <= eps;
    pass &= std::abs(expected.z - actual.z) <= eps;

    EXPECT_TRUE(pass) << msg << "\nExpected: " << formatVec(expected)
                      << "\nActual:   " << formatVec(actual);
}

void TestUtil::AssertVec3Near(const glm::vec3 &expected, const glm::vec3 &actual, std::string msg,
                              float eps)
{
    auto formatVec = [](const glm::vec3 &v)
    {
        std::ostringstream oss;
        oss << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return oss.str();
    };

    bool pass = true;
    pass &= std::abs(expected.x - actual.x) <= eps;
    pass &= std::abs(expected.y - actual.y) <= eps;
    pass &= std::abs(expected.z - actual.z) <= eps;

    ASSERT_TRUE(pass) << msg << "\nExpected: " << formatVec(expected)
                      << "\nActual:   " << formatVec(actual);
}
