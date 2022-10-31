#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <thread>

#include "rviz_manager.h"
#include "slam_lib/utils/math_helpers.h"
#include "tf2_ros/transform_broadcaster.h"

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
    m_transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*m_node_ptr);
}

template <typename FloatT>
void RVizManager<FloatT>::wait_msgs(FloatT delay) const
{
    auto d = std::chrono::milliseconds(static_cast<int>(delay*1000));
    std::this_thread::sleep_for(d);
}

template <typename FloatT>
void RVizManager<FloatT>::publish_map(const OccupancyGrid<FloatT>& occ_grid) const
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
        for (int col = 0; col < occ_grid.width(); ++col)
        {
            FloatT occ_prob = map(row - 1, col);
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
void RVizManager<FloatT>::publish_pose(const Pose<FloatT>& pose) const
{
    geometry_msgs::msg::PoseStamped pose_msg;
    fill_pose_msg(pose, pose_msg.pose);
    pose_msg.header.frame_id = "map";

    // pose_msg.header.stamp = odometry_value.timestamp;
    m_pose_publisher->publish(pose_msg);
}

template <typename FloatT>
void RVizManager<FloatT>::publish_pose_array(const std::vector<Pose<FloatT>>& pose_vector) const
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
void RVizManager<FloatT>::publish_laser_scan(
    const std::vector<FloatT>& laser_scan,
    const LidarConfig<FloatT>& lidar_cfg) const
{
    LaserScanMsg msg;
    msg.range_max = lidar_cfg.max_range();
    msg.range_min = 0;
    msg.angle_min = lidar_cfg.min_angle();
    msg.angle_max = lidar_cfg.max_angle() + lidar_cfg.step();
    msg.angle_increment = lidar_cfg.step();
    for (auto range : laser_scan)
    {
        msg.ranges.push_back(range);
    }

    // don't know why when there is only one value in vector, the value is not showed,
    // the only way to get the range showed is to send a vector of size 2.
    // so copy first value
    if(msg.ranges.size() == 1)
    {
        msg.ranges.push_back(msg.ranges[0]);
    }

    msg.header.frame_id = lidar_cfg.frame_name();
    m_laser_scan_publisher->publish(msg);
}

template <typename FloatT>
void RVizManager<FloatT>::fill_pose_msg(const Pose<FloatT>& pose, geometry_msgs::msg::Pose& pose_msg) const
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

template <typename FloatT>
void RVizManager<FloatT>::publish_transform(const Pose<FloatT>& pose, const std::string& frame_name) const
{
    TranformMsg t;

    // Read message content and assign it to
    // corresponding tf variables
    // t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = frame_name;
    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = pose.x;
    t.transform.translation.y = pose.y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.yaw);
    t.transform.rotation.x = q.getX();
    t.transform.rotation.y = q.getY();
    t.transform.rotation.z = q.getZ();
    t.transform.rotation.w = q.getW();

    // Send the transformation
    m_transform_broadcaster->sendTransform(t);
}

}  // namespace slam