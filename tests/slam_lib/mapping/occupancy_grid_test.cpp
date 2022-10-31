#include "occupancy_grid_test.h"

#include <slam_lib/io/map_reader.h>
#include <slam_lib/mapping/occupancy_grid.h>
#include <slam_lib/ros/rviz_manager.h>
#include <slam_lib/utils/math_helpers.h>

#include <iostream>

TEST_F(OccupancyGridTest, test_frame_transformations)
{
    using FloatT = float;
    using IPose = slam::IPose<FloatT>;
    using Pose = slam::Pose<FloatT>;

    std::string wean_map_path = (RESOURCES_DIR / "test_map.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>::ConstPtr test_map = map_reader.read_map(wean_map_path);

    Pose pose(1.4, 0.8, 0);
    IPose grid_pose = test_map->to_map_frame(pose);
    ASSERT_EQ(14, grid_pose.x);
    ASSERT_EQ(12, grid_pose.y);
    EXPECT_NEAR(0, grid_pose.yaw, PRECISION<FloatT>);

    Pose world_pose = test_map->to_world_frame(grid_pose);
    ASSERT_FLOAT_EQ(pose.x, world_pose.x);
    ASSERT_FLOAT_EQ(pose.y, world_pose.y);
    EXPECT_NEAR(pose.yaw, world_pose.yaw, PRECISION<FloatT>);

    FloatT occ_prob = test_map->map()(grid_pose.y, grid_pose.x);
    ASSERT_FLOAT_EQ(1, occ_prob);

    slam::RVizManager<FloatT> rviz(m_node_ptr);
    rviz.publish_map(*test_map);
    rviz.publish_pose(world_pose);
    rviz.wait_msgs();
}

TEST_F(OccupancyGridTest, test_ray_projection)
{
    using FloatT = float;
    using IPoint = slam::IPoint;
    using IPose = slam::IPose<FloatT>;
    using Pose = slam::Pose<FloatT>;

    std::string wean_map_path = (RESOURCES_DIR / "wean.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>::ConstPtr wean_map = map_reader.read_map(wean_map_path);

    FloatT yaw = M_PI_2;
    FloatT range = 4.5;
    IPose pose1(400, 400, yaw);
    auto point = wean_map->project_ray(pose1, range);

    IPoint expected_point(400, 445);
    ASSERT_EQ(expected_point.x, point.x);
    ASSERT_EQ(expected_point.y, point.y);

    slam::RVizManager<FloatT> rviz(m_node_ptr);
    rviz.publish_map(*wean_map);
    IPose pose2(point.x, point.y, yaw);
    std::vector<Pose> poses{wean_map->to_world_frame(pose1), wean_map->to_world_frame(pose2)};
    rviz.publish_pose_array(poses);
    rviz.wait_msgs();
}

TEST_F(OccupancyGridTest, test_ray_projection2)
{
    using FloatT = float;
    using IPoint = slam::IPoint;
    using IPose = slam::IPose<FloatT>;
    using Pose = slam::Pose<FloatT>;

    std::string wean_map_path = (RESOURCES_DIR / "wean.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>::ConstPtr wean_map = map_reader.read_map(wean_map_path);

    FloatT yaw = M_PI_2;
    FloatT range = 0.42;
    IPose pose1(400, 400, yaw);
    auto point = wean_map->project_ray(pose1, range);

    IPoint expected_point(400, 404);
    ASSERT_EQ(expected_point.x, point.x);
    ASSERT_EQ(expected_point.y, point.y);

    slam::RVizManager<FloatT> rviz(m_node_ptr);
    rviz.publish_map(*wean_map);
    IPose pose2(point.x, point.y, yaw);
    std::vector<Pose> poses{wean_map->to_world_frame(pose1), wean_map->to_world_frame(pose2)};
    rviz.publish_pose_array(poses);
    rviz.wait_msgs();
}

TEST_F(OccupancyGridTest, test_ray_tracing)
{
    using FloatT = float;
    using IPoint = slam::IPoint;
    using IPose = slam::IPose<FloatT>;
    using Pose = slam::Pose<FloatT>;

    std::string wean_map_path = (RESOURCES_DIR / "wean.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>::ConstPtr wean_map = map_reader.read_map(wean_map_path);

    FloatT step = M_PI_2 / 3;
    std::vector<FloatT> expected_values{
        7.5999999,
        0.80621779,
        0.94282037,
        13.2,
        3.00166607,
        1.61243558,
        1.30000007,
        1.38923049,
        2.1954484,
        11.1000004,
        2.3320508,
        1.1660254
    };
    size_t num_scans = expected_values.size();
    std::vector<FloatT> ranges;

    for (size_t i = 0; i < num_scans; ++i)
    {
        FloatT yaw = slam::wrap_to_pi_range(i * step);
        IPose pose1(400, 400, yaw);
        FloatT range = wean_map->ray_tracing(pose1, 0.15, 0.15);
        ranges.push_back(range);
        EXPECT_NEAR(expected_values[i], range, PRECISION<FloatT>);
    }

    // ranges are ordered in wrt map frame, reorder ranges to be in world frame from 0-2pi
    std::reverse(ranges.begin() + 1, ranges.end());

    const std::string name = "lidar";
    auto lidar_cfg = slam::LidarConfigBuilder<FloatT>("lidar")
                         .set_position_offset(0)
                         .set_scan_range(num_scans, 0, 2 * M_PI)
                         .build();
    IPose pose1(400, 400, 0);
    auto lidar_pose_w = wean_map->to_world_frame(pose1);

    slam::RVizManager<FloatT> rviz(m_node_ptr);
    rviz.publish_map(*wean_map);
    rviz.publish_pose(wean_map->to_world_frame(pose1));
    rviz.publish_transform(lidar_pose_w, lidar_cfg.frame_name());
    rviz.publish_laser_scan(ranges, lidar_cfg);
    rviz.wait_msgs();
}

TEST_F(OccupancyGridTest, test_ray_tracing_without_thresholds)
{
    using FloatT = float;
    using IPoint = slam::IPoint;
    using IPose = slam::IPose<FloatT>;
    using Pose = slam::Pose<FloatT>;

    std::string wean_map_path = (RESOURCES_DIR / "wean.dat").string();
    slam::MapReader<FloatT> map_reader;
    slam::OccupancyGrid<FloatT>::ConstPtr wean_map = map_reader.read_map(wean_map_path);

    FloatT yaw = M_PI_2;
    IPose pose1(400, 400, yaw);

    FloatT range = wean_map->ray_tracing(pose1);
    EXPECT_TRUE(std::isnan(range));
}
