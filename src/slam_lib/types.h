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

    PointXY() {}
    PointXY(const T x, const T y) : x(x), y(y) {}

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

template <typename FloatT>
struct Pose2D : PointXY<FloatT>
{
    Pose2D(){}
    Pose2D(const FloatT x, const FloatT y, const FloatT yaw) : PointXY<FloatT>(x, y), yaw(yaw) {}

    FloatT yaw;
};

template<typename T>
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

}  // namespace slam

#endif  // __SLAM_LIB_TYPES_H_
