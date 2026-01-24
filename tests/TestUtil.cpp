#include "TestUtil.hpp"
#include <glm/vec3.hpp>
#include <gtest/gtest.h>

// Boolean version - for conditional checks
bool TestUtil::Vec3Near(const glm::vec3 &expected, const glm::vec3 &actual, float eps)
{
    return std::abs(expected.x - actual.x) <= eps && std::abs(expected.y - actual.y) <= eps &&
           std::abs(expected.z - actual.z) <= eps;
}

void TestUtil::ExpectVec3Near(const glm::vec3 &expected, const glm::vec3 &actual, std::string msg,
                              float eps)
{
    auto formatVec = [](const glm::vec3 &v)
    {
        std::ostringstream oss;
        oss << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return oss.str();
    };

    bool x_match = std::abs(expected.x - actual.x) <= eps;
    bool y_match = std::abs(expected.y - actual.y) <= eps;
    bool z_match = std::abs(expected.z - actual.z) <= eps;
    bool pass    = x_match && y_match && z_match;

    if (!pass)
    {
        GTEST_FAIL() << msg << "\n"
                     << "Expected: " << formatVec(expected) << "\n"
                     << "Actual:   " << formatVec(actual);
    }
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
