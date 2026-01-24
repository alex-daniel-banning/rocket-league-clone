#include <engine/physics/Collisions.hpp>
#include <gtest/gtest.h>
#include "test_util.hpp"

// Base fixture with common utilities
class CollisionResolutionTest : public ::testing::Test
{
  protected:
    // Helper to compute total linear momentum
    glm::vec3 totalLinearMomentum(const engine::physics::Sphere &s, const engine::physics::Box &b)
    {
        return s.mass * s.velocity + b.mass * b.velocity;
    }

    // Helper to compute total angular momentum
    glm::vec3 totalAngularMomentum(const engine::physics::Sphere &s, const engine::physics::Box &b)
    {
        // Orbital angular momentum: L_orbital = r x p
        glm::vec3 sphere_orbital_L = glm::cross(s.position, s.mass * s.velocity);
        glm::vec3 box_orbital_L    = glm::cross(b.position, b.mass * b.velocity);

        // Spin angular momentum: L_spin = I * w
        glm::vec3 sphere_spin_L = glm::vec3(0.0f); // Not implemented yet (no friction)
        glm::vec3 box_spin_L    = b.inertia_tensor * b.angular_velocity;

        return sphere_orbital_L + box_orbital_L + sphere_spin_L + box_spin_L;
    }

    // Helper to compute total kinetic energy
    float totalKineticEnergy(const engine::physics::Sphere &s, const engine::physics::Box &b)
    {
        float linear = 0.5f * s.mass * glm::dot(s.velocity, s.velocity) +
                       0.5f * b.mass * glm::dot(b.velocity, b.velocity);
        // Only box has rotational KE: KE_rot = (1/2) * w^T * I * w
        glm::vec3 Iw     = b.inertia_tensor * b.angular_velocity;
        float rotational = 0.5f * glm::dot(b.angular_velocity, Iw);
        return linear + rotational;
    }
};

// Conservation law tests - these could be parameterized across scenarios
class SphereBox : public CollisionResolutionTest, public ::testing::WithParamInterface<std::string>
{
  protected:
    // Different collision scenarios
    void setupScenario(const std::string &scenario, engine::physics::Sphere &sphere,
                       engine::physics::Box &box)
    {
        if (scenario == "Approaching")
        {
            sphere = engine::physics::Sphere(1.0f, 2.0f, glm::vec3(1.499f, 0.0f, 0.0f),
                                             glm::vec3(-2.0f, 0.0f, 0.0f));
            box    = engine::physics::Box(glm::vec3(1.0f), glm::vec3(0.0f),
                                          glm::vec3(1.0f, 0.0f, 0.0f), 3.0f, glm::quat());
        }
        else if (scenario == "SameDirection")
        {
            sphere = engine::physics::Sphere(1.0f, 2.0f, glm::vec3(1.499f, 0.0f, 0.0f),
                                             glm::vec3(-3.0f, 0.0f, 0.0f));
            box    = engine::physics::Box(glm::vec3(1.0f), glm::vec3(0.0f),
                                          glm::vec3(-1.0f, 0.0f, 0.0f), 3.0f, glm::quat());
        }
        else if (scenario == "OffCenter")
        {
            sphere = engine::physics::Sphere(1.0f, 1.0f, glm::vec3(1.499f, 0.3f, 0.0f),
                                             glm::vec3(-2.0f, 0.0f, 0.0f));
            box    = engine::physics::Box(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(0.0f), 5.0f,
                                          glm::quat());
        }
    }
};

TEST_P(SphereBox, LinearMomentum)
{
    engine::physics::Sphere sphere;
    engine::physics::Box box;
    setupScenario(GetParam(), sphere, box);

    glm::vec3 momentum_before = totalLinearMomentum(sphere, box);

    engine::physics::Contact contact;
    ASSERT_TRUE(engine::physics::Collisions::computeContact(box, sphere, contact));
    engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

    glm::vec3 momentum_after = totalLinearMomentum(sphere, box);

    test_util::ExpectVec3Near(momentum_before, momentum_after, "Linear momentum", 1e-5f);
}

TEST_P(SphereBox, AngularMomentum)
{
    engine::physics::Sphere sphere;
    engine::physics::Box box;
    setupScenario(GetParam(), sphere, box);

    glm::vec3 L_before = totalAngularMomentum(sphere, box);

    engine::physics::Contact contact;
    ASSERT_TRUE(engine::physics::Collisions::computeContact(box, sphere, contact));
    engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

    glm::vec3 L_after = totalAngularMomentum(sphere, box);

    test_util::ExpectVec3Near(L_before, L_after, "Angular momentum", 1e-5f);
}

TEST_P(SphereBox, TotalKineticEnergy)
{
    engine::physics::Sphere sphere;
    engine::physics::Box box;
    setupScenario(GetParam(), sphere, box);

    float KE_before = totalKineticEnergy(sphere, box);

    engine::physics::Contact contact;
    ASSERT_TRUE(engine::physics::Collisions::computeContact(box, sphere, contact));
    engine::physics::Collisions::resolveElasticCollision(box, sphere, contact);

    float KE_after = totalKineticEnergy(sphere, box);

    EXPECT_NEAR(KE_before, KE_after, 1e-4f)
        << "Total kinetic energy should be conserved in elastic collision";
}

INSTANTIATE_TEST_SUITE_P(ConservationLaws, SphereBox,
                         ::testing::Values("Approaching", "SameDirection", "OffCenter"),
                         [](const testing::TestParamInfo<std::string> &info)
                         { return info.param; });
