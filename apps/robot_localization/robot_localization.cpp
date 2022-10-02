

#include <iostream>
#include <thread>

#include "nav_msgs/msg/occupancy_grid.h"
#include "rclcpp/rclcpp.hpp"
#include "slam_lib/io/map_reader.h"

using namespace std::chrono_literals;  // ns, us, ms, s, h, etc.

void robot_localization(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pf_localization");
    auto map_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);

    RCLCPP_INFO_STREAM(node->get_logger(), "PF localization node running. Waiting RVIZ!");

    RCLCPP_INFO_STREAM(node->get_logger(), "RVIZ should be ready.");

    std::string map_file_path = "/home/diego/dev/slam_lib/tests/resources/wean.dat";
    slam::MapReader<float> map_reader;
    slam::OccupancyGrid<float> occupancy_grid = map_reader.read_map(map_file_path);
    // RCLCPP_INFO_STREAM(node->get_logger(), "Map size is " << occupancy_grid.size());

    nav_msgs::msg::OccupancyGrid message = occupancy_grid.to_ros_msg();
    // RCLCPP_INFO_STREAM(node->get_logger(), "Map size is " << message.data.size());

    // message.header.stamp = node->get_clock()->now();

    map_publisher->publish(message);

    RCLCPP_INFO_STREAM(node->get_logger(), "Occupancy grid published!");

    // make sure that the node is not shut down too soon
    std::this_thread::sleep_for(1s);
    rclcpp::shutdown();
}
