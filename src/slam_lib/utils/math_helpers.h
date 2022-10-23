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

#include <algorithm>
#include <vector>
#include <random>

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

/**
 * @brief Normal distribution random number generator
 * 
 * @tparam FloatT floating point type
 * @param mean mean value 
 * @param std_dev standard deviation
 */
template<typename FloatT>
std::vector<FloatT> generate_random_normal_dist(
    const size_t num_values,
    const FloatT mean,
    const FloatT std_dev)
{
    // for initializing random seed
    std::default_random_engine gen(std::random_device{}());

    // Lower and Upper limits are declared in base class
    std::normal_distribution<FloatT> distr(mean, std_dev);

    std::vector<FloatT> values;
    values.reserve(num_values);

    // Compute random numbers
    std::generate_n(std::back_inserter(values), num_values, [&distr, &gen]{ return distr(gen); });

    return values;
}

}  // namespace slam

#endif  // __SLAM_LIB_UTILS_MATH_HELPERS_H_
