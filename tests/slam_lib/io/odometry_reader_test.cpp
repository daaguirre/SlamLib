#include "odometry_reader_test.h"

#include <slam_lib/io/odometry_reader.h>

TEST_F(OdometryReaderTest, should_read_file_float)
{
    using TestType = float;
    const size_t num_scans = 180;
    std::string odometry_log_path = (RESOURCES_DIR / "robotdata1.log").string();
    slam::OdometryReader<TestType> reader;
    slam::RobotOdometry<TestType> odometry_data =
        reader.read_robot_odometry_file(odometry_log_path, num_scans);

    ASSERT_EQ(2218, odometry_data.size());
}