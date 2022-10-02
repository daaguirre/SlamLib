#ifndef __TESTBASE_H_
#define __TESTBASE_H_

#include <gtest/gtest.h>

#include <filesystem>

/**
 * @brief Test base class
 */
class TestBase : public ::testing::Test
{
protected:
    /**
     * @brief precision value for comparing floating point numbers
     */
    static constexpr const double PRECISION = 1e-6;

    /**
     * @brief Path to test resources dir
     */
    static const std::filesystem::path RESOURCES_DIR;
};

#endif  // __TESTBASE_H_
