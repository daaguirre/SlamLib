#include <cmath>

#include "math_helpers.h"

namespace slam
{

template <typename FloatT>
FloatT wrap_to_pi_range(FloatT value)
{
    FloatT new_value = value;
    if (new_value > M_PI)
    {
        new_value -= 2 * M_PI;
    }
    else if (new_value < -M_PI)
    {
        new_value += 2 * M_PI;
    }
    return new_value;
    // return std::fmod(value + M_PI, 2 * M_PI) - M_PI;
}

template <typename FloatT>
Polar<FloatT> to_polar(const FloatT x, const FloatT y)
{
    Polar<FloatT> polar;
    polar.rho = std::sqrt(x * x + y * y);
    polar.theta = std::atan2(y, x);
    return polar;
}

}  // namespace slam