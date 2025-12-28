#include <engine/physics/Plane.hpp>
#include <gtest/gtest.h>

void ExpectVec3Near(const glm::vec3 &a, const glm::vec3 &b, float eps = 1e-5f)
{
    EXPECT_NEAR(a.x, b.x, eps);
    EXPECT_NEAR(a.y, b.y, eps);
    EXPECT_NEAR(a.z, b.z, eps);
}

// todo, remove
TEST(PlaneTest, DefaultInitialization)
{
    engine::physics::Plane p(10.0f, 5.0f, glm::vec3(0.5f));
    EXPECT_EQ(p.getXLength(), 10.0f);
    EXPECT_EQ(p.getZLength(), 5.0f);
    EXPECT_EQ(p.getPosition(), glm::vec3(0.0f));
}

TEST(PlaneTest, PlaneRotation)
{
    glm::vec3 xAxis         = glm::vec3(1.0f, 0.0f, 0.0f);
    float degreesOfRotation = glm::radians(90.0f);
    float xsize = 10.0f, zsize = 5.0f;
    float hx = xsize * 0.5f, hz = zsize * 0.5f;
    engine::physics::Plane p(xsize, zsize, glm::vec3(0.5f), glm::vec3(0.0f),
                             glm::angleAxis(degreesOfRotation, xAxis));
    // clang-format off
    ExpectVec3Near(p.getCornerPositions()[0], glm::vec3(-hx, hz, 0.0f)); 
    ExpectVec3Near(p.getCornerPositions()[1], glm::vec3( hx,  hz, 0.0f));
    ExpectVec3Near(p.getCornerPositions()[2], glm::vec3( hx, -hz, 0.0f));
    ExpectVec3Near(p.getCornerPositions()[3], glm::vec3(-hx, -hz, 0.0f));
    // clang-format on
}

TEST(PlaneTest, PlaneTranslation)
{
    // Plane(float xsize, float zsize, glm::vec3 color);
    glm::vec3 xAxis         = glm::vec3(1.0f, 0.0f, 0.0f);
    float degreesOfRotation = glm::radians(0.0f);
    float xsize = 10.0f, zsize = 5.0f;
    float hx = xsize * 0.5f, hz = zsize * 0.5f;
    glm::vec3 pos = glm::vec3(1.0f, 2.0f, -3.0f);
    engine::physics::Plane p(xsize, zsize, glm::vec3(0.5f), pos,
                             glm::angleAxis(degreesOfRotation, xAxis));
    // clang-format off
    ExpectVec3Near(p.getCornerPositions()[0], pos + glm::vec3(-hx, 0.0f,-hz)); 
    ExpectVec3Near(p.getCornerPositions()[1], pos + glm::vec3( hx, 0.0f,-hz));
    ExpectVec3Near(p.getCornerPositions()[2], pos + glm::vec3( hx, 0.0f, hz));
    ExpectVec3Near(p.getCornerPositions()[3], pos + glm::vec3(-hx, 0.0f, hz));
    // clang-format on
}

TEST(PlaneTest, PlaneRotationAndTranslation)
{
    // Plane(float xsize, float zsize, glm::vec3 color);
    glm::vec3 xAxis         = glm::vec3(1.0f, 0.0f, 0.0f);
    float degreesOfRotation = glm::radians(90.0f);
    float xsize = 10.0f, zsize = 5.0f;
    float hx = xsize * 0.5f, hz = zsize * 0.5f;
    glm::vec3 pos = glm::vec3(1.0f, 2.0f, -3.0f);
    engine::physics::Plane p(xsize, zsize, glm::vec3(0.5f), pos,
                             glm::angleAxis(degreesOfRotation, xAxis));
    // clang-format off
    ExpectVec3Near(p.getCornerPositions()[0], pos + glm::vec3(-hx, hz, 0.0f)); 
    ExpectVec3Near(p.getCornerPositions()[1], pos + glm::vec3( hx,  hz, 0.0f));
    ExpectVec3Near(p.getCornerPositions()[2], pos + glm::vec3( hx, -hz, 0.0f));
    ExpectVec3Near(p.getCornerPositions()[3], pos + glm::vec3(-hx, -hz, 0.0f));
    // clang-format on
}
