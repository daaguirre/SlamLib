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
static const FloatT ONE_OVER_SQRT_2PI = 0.39894228040143267793994605993438;

template <typename FloatT>
inline FloatT squared(FloatT x)
{
    return x * x;
}

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

template <typename FloatT>
FloatT normpdf(FloatT x, FloatT mu, FloatT std_dev)
{
    return (ONE_OVER_SQRT_2PI<FloatT> / std_dev) * std::exp(-0.5 * squared((x - mu) / std_dev));
}

}  // namespace slam

#endif  // __SLAM_LIB_UTILS_MATH_HELPERS_H_
