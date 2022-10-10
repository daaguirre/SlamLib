#include <fstream>

#include "odometry_reader.h"

namespace slam
{

template <typename T>
RobotOdometry<T> slam::OdometryReader<T>::read_robot_odometry_file(
    const std::string& file_path,
    const size_t num_scans)
{
    std::ifstream file(file_path);
    if (file.fail())
    {
        std::stringstream error_stream;
        error_stream << "Input file " << file_path << " not found\n";
        throw std::invalid_argument(error_stream.str());
    }

    RobotOdometry<T> robot_odometry;
    std::string line;
    while (std::getline(file, line))
    {
        LidarOdometry<T> lidar_odometry = decode_line(line, num_scans);
        robot_odometry.push_back(lidar_odometry);
    }

    return robot_odometry;
}

template <typename T>
LidarOdometry<T> slam::OdometryReader<T>::decode_line(
    const std::string& line,
    const size_t num_scans)
{
    LidarOdometry<T> odometry;

    std::istringstream iss(line);
    char reading_type;
    iss >> reading_type;
    iss >> odometry.x >> odometry.y >> odometry.yaw;
    odometry.x *= 0.1;
    odometry.y *= 0.1;
    if (reading_type == 'L')
    {
        iss >> odometry.lx >> odometry.ly >> odometry.lyaw;
        odometry.lx *= 0.1;
        odometry.ly *= 0.1;

        odometry.ranges.resize(num_scans, 1);
        T value;
        for (size_t i = 0; i < num_scans; ++i)
        {
            iss >> value;
            odometry.ranges(i) = value * 0.1;
        }
    }

    iss >> odometry.timestamp;
    return odometry;
}

}  // namespace slam