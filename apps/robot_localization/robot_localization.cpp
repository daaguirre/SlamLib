

#include <geometry_msgs/msg/transform.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <slam_lib/localization/particle_filter.h>
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
using FloatT = float;

void robot_localization(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pf_localization");

    RCLCPP_INFO_STREAM(node->get_logger(), "Starting particle filter...");

    std::string map_file_path = "/home/diego/dev/slam_lib/tests/resources/wean.dat";
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>::Ptr map_ptr = map_reader.read_map(map_file_path);

    const size_t num_particles = 3000;
    slam::ParticleFilter<FloatT> particle_filter(num_particles, map_ptr);

    auto lidar_cfg_ptr = slam::LidarConfigBuilder<FloatT>("lidar")
                             .set_position_offset(0.25)
                             .set_scan_range(180, -M_PI_2, M_PI_2)
                             .build_as_shared();

    slam::OdometryReader<FloatT> odom_reader;
    slam::RobotOdometry<FloatT> robot_odometry = odom_reader.read_robot_odometry_file(
        "/home/diego/dev/slam_lib/tests/resources/robotdata1.log", lidar_cfg_ptr);

    slam::RVizManager<FloatT> rviz(node);
    rviz.publish_map(*map_ptr);
    rviz.wait_msgs();

    RCLCPP_INFO_STREAM(node->get_logger(), "Running particle filter...");

    for (size_t i = 0; i < robot_odometry.size(); ++i)
    {
        if (i > 0)
        {
            RCLCPP_INFO_STREAM(node->get_logger(), "odometry update: " << i);

            const auto& previous_reading = robot_odometry[i - 1];
            const auto& current_reading = robot_odometry[i];
            auto robot_grid_pose = particle_filter.update(previous_reading, current_reading);

            auto robot_world_pose = map_ptr->to_world_frame(robot_grid_pose);
            rviz.publish_pose(robot_world_pose);
        }
        const auto& particles = particle_filter.get_particles();

        std::vector<slam::Pose<FloatT>> poses(particles.size());
        std::transform(
            particles.begin(),
            particles.end(),
            poses.begin(),
            [map_ptr](const auto& particle) { return map_ptr->to_world_frame(particle); });

        rviz.publish_pose_array(poses);
    }

    rclcpp::shutdown();
}
