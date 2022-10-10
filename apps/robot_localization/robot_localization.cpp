

#include <nav_msgs/msg/occupancy_grid.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "slam_lib/io/map_reader.h"
#include "slam_lib/io/odometry_reader.h"

using namespace std::chrono_literals;  // ns, us, ms, s, h, etc.

void robot_localization(int argc, char** argv)
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

    // nav_msgs::msg::OccupancyGrid message = occupancy_grid.to_ros_msg();
    // // RCLCPP_INFO_STREAM(node->get_logger(), "Map size is " << message.data.size());

    // // message.header.stamp = node->get_clock()->now();

    // map_publisher->publish(message);

    // RCLCPP_INFO_STREAM(node->get_logger(), "Occupancy grid published!");

    // auto pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 1);
    // slam::OdometryReader<float> odom_reader;
    // slam::RobotOdometry<float> robot_odometry = odom_reader.read_robot_odometry_file(
    //     "/home/diego/dev/slam_lib/tests/resources/robotdata1.log", 180);

    // for (auto& odometry_value : robot_odometry)
    // {
    //     geometry_msgs::msg::PoseStamped pose_msg;
    //     pose_msg.header.frame_id = "map";
    //     // pose_msg.header.stamp = odometry_value.timestamp;
    //     pose_msg.pose.position.x = static_cast<double>(odometry_value.x);
    //     pose_msg.pose.position.y = static_cast<double>(odometry_value.y);
    //     pose_msg.pose.position.z = 0;
    //     tf2::Quaternion orientation_q;
    //     orientation_q.setRPY(0, 0, odometry_value.yaw);
    //     geometry_msgs::msg::Quaternion q_msg;
    //     q_msg.x = orientation_q.getX();
    //     q_msg.y = orientation_q.getY();
    //     q_msg.z = orientation_q.getZ();
    //     q_msg.w = orientation_q.getW();
    //     pose_msg.pose.orientation = q_msg;
    //     pose_publisher->publish(pose_msg);
    //     std::this_thread::sleep_for(0.2s);
    // }

    // // make sure that the node is not shut down too soon
    // std::this_thread::sleep_for(1s);
    // rclcpp::shutdown();
}
