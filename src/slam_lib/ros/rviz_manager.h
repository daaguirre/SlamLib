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

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "slam_lib/mapping/occupancy_grid.h"
#include "slam_lib/odometry/odometry_models.h"
#include "tf2_ros/transform_broadcaster.h"

namespace slam
{

template <typename FloatT>
class RVizManager
{
public:
    using MapMsg = nav_msgs::msg::OccupancyGrid;
    using MapPublisher = rclcpp::Publisher<MapMsg>;

    using PoseMsg = geometry_msgs::msg::PoseStamped;
    using PosePublisher = rclcpp::Publisher<PoseMsg>;

    using PoseArrayMsg = geometry_msgs::msg::PoseArray;
    using PoseArrayPublisher = rclcpp::Publisher<PoseArrayMsg>;

    using LaserScanMsg = sensor_msgs::msg::LaserScan;
    using LaserScanPublisher = rclcpp::Publisher<LaserScanMsg>;

    using TransformBroadcaster = tf2_ros::TransformBroadcaster;
    using TranformMsg = geometry_msgs::msg::TransformStamped;

    RVizManager(rclcpp::Node::SharedPtr node_ptr);

    void publish_map(const OccupancyGrid<FloatT>& occ_grid) const;
    void publish_pose(const Pose<FloatT>& pose) const ;
    void publish_pose_array(const std::vector<Pose<FloatT>>& pose_vector) const;
    void publish_laser_scan(
        const std::vector<FloatT>& ranges,
        const LidarConfig<FloatT>& lidar_cfg) const;
    void publish_transform(const Pose<FloatT>& pose, const std::string& frame_name) const;

    void wait_msgs() const;

private:
    void fill_pose_msg(const Pose<FloatT>& pose, geometry_msgs::msg::Pose& pose_msg) const;

    rclcpp::Node::SharedPtr m_node_ptr = nullptr;
    MapPublisher::SharedPtr m_map_publisher = nullptr;
    PosePublisher::SharedPtr m_pose_publisher = nullptr;
    LaserScanPublisher::SharedPtr m_laser_scan_publisher = nullptr;
    PoseArrayPublisher::SharedPtr m_pose_array_publisher = nullptr;
    std::unique_ptr<TransformBroadcaster> m_transform_broadcaster = nullptr;
};

}  // namespace slam

#endif  // __SLAM_LIB_ROS_ROS_MANAGER_H_
