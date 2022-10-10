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

template<typename FloatT>
using Vector = Eigen::Matrix<FloatT, -1, 1>;

template <typename T>
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
    Vector<T> ranges;
};


template<typename T>
struct RobotOdometry : public std::vector<LidarOdometry<T>>
{
    Vector<T> scan_angles;
};

}  // namespace slam

#endif  // __SLAM_LIB_ODOMETRY_ROBOT_ODOMETRY_H_
