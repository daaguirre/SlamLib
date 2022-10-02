/**
 * @file log_reader.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SLAM_LIB_IO_LOG_READER_H_
#define __SLAM_LIB_IO_LOG_READER_H_


#include "slam_lib/odometry/odometry_models.h"


namespace slam
{

template<typename T>
class OdometryReader
{
public:
    OdometryReader() = default;

    RobotOdometry<T> read_robot_odometry_file(const std::string& file_path, const size_t num_scans);

private:
    LidarOdometry<T> decode_line(const std::string& line, const size_t num_scans);
};

}  // namespace slam

#endif  // __SLAM_LIB_IO_LOG_READER_H_
