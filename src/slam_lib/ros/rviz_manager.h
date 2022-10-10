/**
 * @file ros_manager.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SLAM_LIB_ROS_ROS_MANAGER_H_
#define __SLAM_LIB_ROS_ROS_MANAGER_H_

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "slam_lib/mapping/occupancy_grid.h"

namespace slam
{

template <typename FloatT>
class RVizManager
{
public:
    using Point = typename OccupancyGrid<FloatT>::Point;
    using Pose = typename OccupancyGrid<FloatT>::Pose;

    using MapMsg = nav_msgs::msg::OccupancyGrid;
    using MapPublisher = rclcpp::Publisher<MapMsg>;

    using PoseMsg = geometry_msgs::msg::PoseStamped;
    using PosePublisher = rclcpp::Publisher<PoseMsg>;

    using PoseArrayMsg = geometry_msgs::msg::PoseArray;
    using PoseArrayPublisher = rclcpp::Publisher<PoseArrayMsg>;

    using LaserScanMsg = sensor_msgs::msg::LaserScan;
    using LaserScanPublisher = rclcpp::Publisher<LaserScanMsg>;

    RVizManager(rclcpp::Node::SharedPtr node_ptr);

    void publish_map(const OccupancyGrid<FloatT>& occ_grid);
    void publish_pose(const Pose& pose);
    void publish_pose_array(const std::vector<Pose>& pose_vector);
    void publish_laser_scan(const std::vector<FloatT>& ranges, const FloatT yaw);

    void wait_msgs();

private:
    void fill_pose_msg(const Pose& pose, geometry_msgs::msg::Pose& pose_msg);

    rclcpp::Node::SharedPtr m_node_ptr = nullptr;
    MapPublisher::SharedPtr m_map_publisher = nullptr;
    PosePublisher::SharedPtr m_pose_publisher = nullptr;
    LaserScanPublisher::SharedPtr m_laser_scan_publisher = nullptr;
    PoseArrayPublisher::SharedPtr m_pose_array_publisher = nullptr;
};

}  // namespace slam

#endif  // __SLAM_LIB_ROS_ROS_MANAGER_H_
