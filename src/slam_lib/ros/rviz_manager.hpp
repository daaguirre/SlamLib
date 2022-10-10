#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>

#include "rviz_manager.h"

namespace slam
{

using namespace std::chrono_literals;  // ns, us, ms, s, h, etc.

template <typename FloatT>
RVizManager<FloatT>::RVizManager(rclcpp::Node::SharedPtr node_ptr) : m_node_ptr(node_ptr)
{
    m_map_publisher = m_node_ptr->create_publisher<MapMsg>("/map", 1);
    m_pose_publisher = m_node_ptr->create_publisher<PoseMsg>("/robot_pose", 1);
    m_pose_array_publisher = m_node_ptr->create_publisher<PoseArrayMsg>("/particle_cloud", 1);
    m_laser_scan_publisher = m_node_ptr->create_publisher<LaserScanMsg>("/laser_scan", 1);
}

template <typename FloatT>
void RVizManager<FloatT>::wait_msgs()
{
    std::this_thread::sleep_for(0.2s);
}

template <typename FloatT>
void RVizManager<FloatT>::publish_map(const OccupancyGrid<FloatT>& occ_grid)
{
    MapMsg occ_grid_msg;
    occ_grid_msg.info.width = occ_grid.width();
    occ_grid_msg.info.height = occ_grid.height();
    occ_grid_msg.info.resolution = occ_grid.resolution();
    occ_grid_msg.header.frame_id = "map";
    occ_grid_msg.info.origin.position.x = 0;
    occ_grid_msg.info.origin.position.y = 0;
    occ_grid_msg.info.origin.position.z = 0.0;
    occ_grid_msg.info.origin.orientation.x = 0.0;
    occ_grid_msg.info.origin.orientation.y = 0.0;
    occ_grid_msg.info.origin.orientation.z = 0.0;
    occ_grid_msg.info.origin.orientation.w = 1.0;
    const auto& map = occ_grid.map();

    for (int row = occ_grid.height(); row > 0; --row)
    {
        for(int col = 0; col < occ_grid.width(); ++col)
        {
            FloatT occ_prob = map(row-1, col);
            if (occ_prob >= 0)
            {
                occ_grid_msg.data.push_back(occ_prob * 100);
            }
            else
            {
                occ_grid_msg.data.push_back(occ_prob);
            }
        }
    }

    // for (int i = 0; i < occ_grid.size(); i++)
    // {
    //     FloatT occ_prob = map.data()[i];
    //     if (occ_prob >= 0)
    //     {
    //         occ_grid_msg.data.push_back(occ_prob * 100);
    //     }
    //     else
    //     {
    //         occ_grid_msg.data.push_back(occ_prob);
    //     }
    // }

    m_map_publisher->publish(occ_grid_msg);
}

template <typename FloatT>
void RVizManager<FloatT>::publish_pose(const Pose& pose)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    fill_pose_msg(pose, pose_msg.pose);
    pose_msg.header.frame_id = "map";

    // pose_msg.header.stamp = odometry_value.timestamp;
    m_pose_publisher->publish(pose_msg);
}

template <typename FloatT>
void RVizManager<FloatT>::publish_pose_array(const std::vector<Pose>& pose_vector)
{
    PoseArrayMsg msg;
    for (auto& pose : pose_vector)
    {
        geometry_msgs::msg::Pose pose_msg;
        fill_pose_msg(pose, pose_msg);
        msg.poses.push_back(pose_msg);
    }
    msg.header.frame_id = "map";

    m_pose_array_publisher->publish(msg);
}

template <typename FloatT>
void RVizManager<FloatT>::publish_laser_scan(const std::vector<FloatT>& laser_scan, const FloatT yaw)
{
    LaserScanMsg msg;
    msg.range_max = 1000;
    msg.range_min = 0;
    msg.angle_min = yaw;
    msg.angle_max = yaw + 0.5;
    msg.angle_increment = 0.2;
    // msg.angle_increme
    for (auto range : laser_scan)
    {
        msg.ranges.push_back(range);
    }

    msg.header.frame_id = "map";
    m_laser_scan_publisher->publish(msg);
}

template <typename FloatT>
void RVizManager<FloatT>::fill_pose_msg(const Pose& pose, geometry_msgs::msg::Pose& pose_msg)
{
    // pose_msg.header.frame_id = "map";
    // pose_msg.header.stamp = odometry_value.timestamp;
    pose_msg.position.x = static_cast<double>(pose.x);
    pose_msg.position.y = static_cast<double>(pose.y);
    pose_msg.position.z = 0;
    tf2::Quaternion orientation_q;
    orientation_q.setRPY(0, 0, pose.yaw);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = orientation_q.getX();
    q_msg.y = orientation_q.getY();
    q_msg.z = orientation_q.getZ();
    q_msg.w = orientation_q.getW();
    pose_msg.orientation = q_msg;
}

}  // namespace slam