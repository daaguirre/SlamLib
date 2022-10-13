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

    robot_odometry.scan_angles.resize(num_scans);
    T angle_res = M_PI / static_cast<T>(num_scans-1);
    T angle = -M_PI_2;
    robot_odometry.scan_angles[0] = angle;
    for (size_t i = 1; i < num_scans; ++i)
    {
        angle += angle_res;
        robot_odometry.scan_angles[i] = angle;
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
    odometry.x *= 0.01;
    odometry.y *= 0.01;
    if (reading_type == 'L')
    {
        iss >> odometry.lx >> odometry.ly >> odometry.lyaw;
        odometry.lx *= 0.01;
        odometry.ly *= 0.01;

        odometry.ranges.resize(num_scans, 1);
        T value;
        for (size_t i = 0; i < num_scans; ++i)
        {
            iss >> value;
            odometry.ranges(i) = value * 0.01;
        }
    }

    iss >> odometry.timestamp;
    return odometry;
}

}  // namespace slam