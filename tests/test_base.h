#ifndef __TESTBASE_H_
#define __TESTBASE_H_

#include <gtest/gtest.h>

#include <filesystem>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Test base class
 */
class TestBase : public ::testing::Test
{
public:
    TestBase();
    ~TestBase();

protected:
    /**
     * @brief precision value for comparing floating point numbers
     */
    template<typename FloatT>
    static constexpr const FloatT PRECISION = 1e-6;

    /**
     * @brief Path to test resources dir
     */
    static const std::filesystem::path RESOURCES_DIR;

    rclcpp::Node::SharedPtr m_node_ptr;
};

#endif  // __TESTBASE_H_
