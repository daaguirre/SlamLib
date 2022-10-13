#include "occupancy_grid_test.h"

#include <slam_lib/io/map_reader.h>
#include <slam_lib/mapping/occupancy_grid.h>
#include <slam_lib/ros/rviz_manager.h>
#include <slam_lib/utils/math_helpers.h>

#include <iostream>

TEST_F(OccupancyGridTest, test_frame_transformations)
{
    using FloatT = float;
    using Point = slam::OccupancyGrid<FloatT>::Point;
    using GridPose = slam::OccupancyGrid<FloatT>::GridPose;
    using Pose = slam::OccupancyGrid<FloatT>::Pose;

    std::string wean_map_path = (RESOURCES_DIR / "test_map.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>& test_map = *map_reader.read_map(wean_map_path);

    Pose pose(1.4, 0.8, 0);
    GridPose grid_pose = test_map.to_map_frame(pose);
    ASSERT_EQ(14, grid_pose.x);
    ASSERT_EQ(12, grid_pose.y);
    EXPECT_NEAR(M_PI, grid_pose.yaw, PRECISION<FloatT>);

    Pose world_pose = test_map.to_world_frame(grid_pose);
    ASSERT_FLOAT_EQ(pose.x, world_pose.x);
    ASSERT_FLOAT_EQ(pose.y, world_pose.y);
    EXPECT_NEAR(pose.yaw, world_pose.yaw, PRECISION<FloatT>);

    FloatT occ_prob = test_map.map()(grid_pose.y, grid_pose.x);
    ASSERT_FLOAT_EQ(1, occ_prob);

    slam::RVizManager<FloatT> rviz(m_node_ptr);
    rviz.publish_map(test_map);
    rviz.publish_pose(world_pose);
    rviz.wait_msgs();
}

TEST_F(OccupancyGridTest, test_ray_projection)
{
    using FloatT = float;
    using Point = slam::OccupancyGrid<FloatT>::Point;
    using GridPose = slam::OccupancyGrid<FloatT>::GridPose;
    using Pose = slam::OccupancyGrid<FloatT>::Pose;

    std::string wean_map_path = (RESOURCES_DIR / "wean.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>& wean_map = *map_reader.read_map(wean_map_path);

    FloatT yaw = M_PI_2;
    FloatT range = 4.5;
    GridPose pose1(400, 400, yaw);
    auto point = wean_map.project_ray(pose1, range);

    Point expected_point(400, 445);
    ASSERT_EQ(expected_point.x, point.x);
    ASSERT_EQ(expected_point.y, point.y);

    slam::RVizManager<FloatT> rviz(m_node_ptr);
    rviz.publish_map(wean_map);
    GridPose pose2(point.x, point.y, yaw);
    std::vector<Pose> poses{wean_map.to_world_frame(pose1), wean_map.to_world_frame(pose2)};
    rviz.publish_pose_array(poses);
    rviz.wait_msgs();
}

TEST_F(OccupancyGridTest, test_ray_projection2)
{
    using FloatT = float;
    using Point = slam::OccupancyGrid<FloatT>::Point;
    using GridPose = slam::OccupancyGrid<FloatT>::GridPose;
    using Pose = slam::OccupancyGrid<FloatT>::Pose;

    std::string wean_map_path = (RESOURCES_DIR / "wean.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>& wean_map = *map_reader.read_map(wean_map_path);

    FloatT yaw = M_PI_2;
    FloatT range = 0.42;
    GridPose pose1(400, 400, yaw);
    auto point = wean_map.project_ray(pose1, range);

    // because of ceil the expected x is 405 instead of 404
    Point expected_point(400, 405);
    ASSERT_EQ(expected_point.x, point.x);
    ASSERT_EQ(expected_point.y, point.y);

    slam::RVizManager<FloatT> rviz(m_node_ptr);
    rviz.publish_map(wean_map);
    GridPose pose2(point.x, point.y, yaw);
    std::vector<Pose> poses{wean_map.to_world_frame(pose1), wean_map.to_world_frame(pose2)};
    rviz.publish_pose_array(poses);
    rviz.wait_msgs();
}

TEST_F(OccupancyGridTest, test_ray_tracing)
{
    using FloatT = float;
    using Point = slam::OccupancyGrid<FloatT>::Point;
    using GridPose = slam::OccupancyGrid<FloatT>::GridPose;
    using Pose = slam::OccupancyGrid<FloatT>::Pose;

    std::string wean_map_path = (RESOURCES_DIR / "wean.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>& wean_map = *map_reader.read_map(wean_map_path);

    FloatT yaw = M_PI_2;
    GridPose pose1(400, 400, yaw);
    FloatT range = wean_map.ray_tracing(pose1);
    FloatT expected_range{13.1};
    EXPECT_NEAR(expected_range, range, PRECISION<FloatT>);
    
    Point point = wean_map.project_ray(pose1, range);
    GridPose pose2(point.x, point.y, pose1.yaw);
    Pose pose_2d = wean_map.to_world_frame(pose2);
    slam::Polar<FloatT> polar = slam::to_polar(pose_2d.x, pose_2d.y);  

    slam::RVizManager<FloatT> rviz(m_node_ptr);
    rviz.publish_map(wean_map);
    rviz.publish_pose(wean_map.to_world_frame(pose1));
    // rviz.publish_laser_scan({polar.rho, polar.rho}, polar.theta);
    rviz.wait_msgs();
}
