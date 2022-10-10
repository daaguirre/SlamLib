#include "test_base.h"

#include <thread>

const std::filesystem::path TestBase::RESOURCES_DIR = std::filesystem::path(RESOURCES_DIR_PATH);

using namespace std::chrono_literals;  // ns, us, ms, s, h, etc.

TestBase::TestBase()
{
    rclcpp::init(0, nullptr);
    m_node_ptr = rclcpp::Node::make_shared("slam_lib_tests");
}

TestBase::~TestBase()
{
    rclcpp::shutdown();
}
