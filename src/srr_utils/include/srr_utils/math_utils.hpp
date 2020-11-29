#ifndef __SRR_MATH_UTILS_HPP__
#define __SRR_MATH_UTILS_HPP__

#include <cmath>

namespace SRR {

double pi()
{
    return M_PI;
}

double rads(double degrees)
{
    return degrees / 180.0 * pi();
}
double degrees(double rads)
{
    return rads * 180.0 / pi();
}

double mag(double x, double y, double z)
{
    return std::sqrt((x * x) + (y * y) + (z * z));
}

}	// namespace SRR

#endif	// __SRR_MATH_UTILS_HPP__
