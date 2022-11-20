/**
 * @file types.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SLAM_LIB_TYPES_H_
#define __SLAM_LIB_TYPES_H_

#include <Eigen/Eigen>

namespace slam
{

template <typename T>
struct PointXY
{
    using Vector = Eigen::Map<Eigen::Matrix<T, 2, 1>>;
    using ConstVector = Eigen::Map<const Eigen::Matrix<T, 2, 1>>;

    PointXY(const T x=0, const T y=0) : x(x), y(y) {}

    union
    {
        T data[2];
        struct
        {
            T x;
            T y;
        };
    };

    ConstVector vector() const
    {
        return ConstVector(data);
    }

    Vector vector()
    {
        return Vector(data);
    }
};

template <typename T, typename YawT>
struct Pose2D : PointXY<T>
{
    Pose2D() {}
    Pose2D(const T x, const T y, const YawT yaw) : PointXY<T>(x, y), yaw(yaw) {}

    YawT yaw{0};
};

template <
    typename FloatT = float,
    typename = std::enable_if_t<std::is_floating_point<FloatT>::value, void>>
using IPose = Pose2D<int, FloatT>;

template <typename T>
struct Box
{
    T min_x = std::numeric_limits<T>::max();
    T min_y = std::numeric_limits<T>::max();
    T max_x = std::numeric_limits<T>::min();
    T max_y = std::numeric_limits<T>::min();

    T width() const
    {
        return max_x - min_x;
    }

    T height() const
    {
        return max_y - min_y;
    }
};

/**
 * @brief integer point. To be used on grids
 *
 */
using IPoint = PointXY<int>;

/**
 * @brief defines the slam::Point as a floating point type
 * by default float
 * @tparam FloatT
 */
template <
    typename FloatT = float,
    typename = std::enable_if_t<std::is_floating_point<FloatT>::value, void>>
using Point = PointXY<FloatT>;

template <
    typename FloatT = float,
    typename = std::enable_if_t<std::is_floating_point<FloatT>::value, void>>
using Pose = Pose2D<FloatT, FloatT>;

}  // namespace slam

#endif  // __SLAM_LIB_TYPES_H_
