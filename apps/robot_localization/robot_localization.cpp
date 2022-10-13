

#include <geometry_msgs/msg/transform.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <slam_lib/particle_filter.h>
#include <slam_lib/ros/rviz_manager.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

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

    RCLCPP_INFO_STREAM(node->get_logger(), "Starting particle filter...");

    std::string map_file_path = "/home/diego/dev/slam_lib/tests/resources/wean.dat";
    slam::MapReader<float> map_reader;
    slam::OccupancyGrid<float>::Ptr map_ptr = map_reader.read_map(map_file_path);

    const size_t num_particles = 3000;
    slam::ParticleFilter<float> particle_filter(num_particles, map_ptr);

    slam::OdometryReader<float> odom_reader;
    slam::RobotOdometry<float> robot_odometry = odom_reader.read_robot_odometry_file(
        "/home/diego/dev/slam_lib/tests/resources/robotdata1.log", 180);

    slam::RVizManager<float> rviz(node);
    rviz.publish_map(*map_ptr);
    rviz.wait_msgs();

    RCLCPP_INFO_STREAM(node->get_logger(), "Running particle filter...");

    for (size_t i = 0; i < robot_odometry.size(); ++i)
    {
        if (i > 0)
        {
            RCLCPP_INFO_STREAM(node->get_logger(), "odometry update: " << i);

            const auto& previous_odom = robot_odometry[i - 1];
            const auto& current_odom = robot_odometry[i];
            auto robot_pose_g =
                particle_filter.update(previous_odom, current_odom, robot_odometry.scan_angles);

            auto robot_pose_w = map_ptr->to_world_frame(robot_pose_g);
            rviz.publish_pose(robot_pose_w);
            // rviz.publish_transform(robot_pose_w);
            // if (current_odom.ranges.size())
            // {
            //     std::vector<float> ranges(
            //         current_odom.ranges.data(),
            //         current_odom.ranges.data() + current_odom.ranges.size());
            //     rviz.publish_laser_scan(ranges, robot_pose_w.yaw);
            // }
        }
        const auto& particles = particle_filter.get_particles();
        // const auto& scan_positions = particle_filter.scan_positions();

        std::vector<slam::OccupancyGrid<float>::Pose> poses(particles.size());
        std::transform(
            particles.begin(),
            particles.end(),
            poses.begin(),
            [map_ptr](const auto& particle) { return map_ptr->to_world_frame(particle); });

        rviz.publish_pose_array(poses);
    }

    rclcpp::shutdown();
}
