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

}	// namespace SRR

#endif	// __SRR_MATH_UTILS_HPP__
