#include <gtest/gtest.h>
#include <iostream>
#include <string>

class CustomPrinter : public ::testing::EmptyTestEventListener
{
    void OnTestStart(const ::testing::TestInfo &test_info) override
    {
        std::cout << "\n"
                  << std::string(100, '=') << "\n"
                  << "RUNNING: " << test_info.test_suite_name() << "." << test_info.name() << "\n"
                  << std::string(100, '=') << std::endl;
    }

    void OnTestEnd(const ::testing::TestInfo &test_info) override
    {
        if (test_info.result()->Failed())
        {
            std::cout << "FAILED: " << test_info.test_suite_name() << "." << test_info.name()
                      << std::endl;
        }
        else
        {
            std::cout << "PASSED: " << test_info.test_suite_name() << "." << test_info.name()
                      << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::UnitTest::GetInstance()->listeners().Append(new CustomPrinter);
    return RUN_ALL_TESTS();
}
