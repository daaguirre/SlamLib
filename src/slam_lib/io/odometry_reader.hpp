#include <fstream>

#include "odometry_reader.h"

namespace slam
{

template <typename FloatT>
RobotOdometry<FloatT> slam::OdometryReader<FloatT>::read_robot_odometry_file(
    const std::string& file_path,
    typename LidarConfig<FloatT>::ConstPtr lidar_cfg_ptr)
{
    std::ifstream file(file_path);
    if (file.fail())
    {
        std::stringstream error_stream;
        error_stream << "Input file " << file_path << " not found\n";
        throw std::invalid_argument(error_stream.str());
    }

    const size_t num_scans = lidar_cfg_ptr->num_scans();

    RobotOdometry<FloatT> robot_odometry;
    std::string line;
    while (std::getline(file, line))
    {
        RobotReading<FloatT>&& lidar_odometry = decode_line(line, num_scans);
        lidar_odometry.lidar_cfg_ptr = lidar_cfg_ptr;
        robot_odometry.push_back(lidar_odometry);
    }

    return robot_odometry;
}

template <typename FloatT>
RobotReading<FloatT> slam::OdometryReader<FloatT>::decode_line(
    const std::string& line,
    const size_t num_scans)
{
    RobotReading<FloatT> reading;
    std::istringstream iss(line);
    char reading_type;
    iss >> reading_type;
    iss >> reading.x >> reading.y >> reading.yaw;
    reading.x *= 0.01;
    reading.y *= 0.01;
    if (reading_type == 'L')
    {
        iss >> reading.lx >> reading.ly >> reading.lyaw;
        reading.lx *= 0.01;
        reading.ly *= 0.01;

        reading.ranges.resize(num_scans, 1);
        FloatT value;
        for (size_t i = 0; i < num_scans; ++i)
        {
            iss >> value;
            reading.ranges(i) = value * 0.01;
        }
    }

    iss >> reading.timestamp;
    return reading;
}

}  // namespace slam