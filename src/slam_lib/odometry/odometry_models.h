/**
 * @file robot_odometry.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SLAM_LIB_ODOMETRY_ROBOT_ODOMETRY_H_
#define __SLAM_LIB_ODOMETRY_ROBOT_ODOMETRY_H_

#include <Eigen/Eigen>

namespace slam
{

template<typename T>
struct PositionOdometry
{
    T x;
    T y;
    T yaw;
    T timestamp;
};

/**
 * @brief Represents the odometry of a robot for a
 * a single position.
 */
template<typename T>
struct LidarOdometry : public PositionOdometry<T>
{
    T lx;
    T ly;
    T lyaw;
    Eigen::Matrix<T, -1, 1> ranges;
};


template<typename T>
struct RobotOdometry : public std::vector<LidarOdometry<T>>
{
    Eigen::Matrix<T, -1, 1> scan_angles;
};

}  // namespace slam

#endif  // __SLAM_LIB_ODOMETRY_ROBOT_ODOMETRY_H_
