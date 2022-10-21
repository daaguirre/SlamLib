#include "odometry_reader_test.h"

#include <slam_lib/io/odometry_reader.h>

TEST_F(OdometryReaderTest, should_read_file_float)
{
    using FloatT = float;
    using RobotReading = slam::RobotReading<FloatT>;
    using RobotOdometry = slam::RobotOdometry<FloatT>;
    using LidarConfig = slam::LidarConfig<FloatT>;
    using Point = slam::Point<FloatT>;

    const size_t num_scans = 180;
    const FloatT min_angle = -M_PI_2;
    const FloatT range = M_PI;
    const std::string lidar_name = "lidar";
    const FloatT max_range = 120;
    const Point offset{0, 0.25};
    auto lidar_cfg_ptr = slam::LidarConfigBuilder<FloatT>("lidar")
                             .set_position_offset(0.25)
                             .set_scan_range(180, -M_PI_2, M_PI_2)
                             .build_as_shared();

    std::string odometry_log_path = (RESOURCES_DIR / "robotdata1.log").string();
    slam::OdometryReader<FloatT> reader;
    RobotOdometry robot_odometry =
        reader.read_robot_odometry_file(odometry_log_path, lidar_cfg_ptr);

    ASSERT_EQ(2218, robot_odometry.size());
}