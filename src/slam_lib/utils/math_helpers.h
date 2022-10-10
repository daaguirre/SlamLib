/**
 * @file math_helpers.h
 * @author daaguirre
 * @brief
 * @date 2022-08-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __SLAM_LIB_UTILS_MATH_HELPERS_H_
#define __SLAM_LIB_UTILS_MATH_HELPERS_H_

namespace slam
{

template <typename FloatT>
struct Polar
{
    FloatT rho;
    FloatT theta;
};

template <typename FloatT>
FloatT wrap_to_pi_range(FloatT value);

template <typename FloatT>
Polar<FloatT> to_polar(const FloatT x, const FloatT y);

}  // namespace slam

#endif  // __SLAM_LIB_UTILS_MATH_HELPERS_H_
