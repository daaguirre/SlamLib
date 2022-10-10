#include "math_helpers.hpp"

namespace slam
{

template float wrap_to_pi_range(float value);
template double wrap_to_pi_range(double value);

template Polar<float> to_polar(const float x, const float y);
template Polar<double> to_polar(const double x, const double y);

}