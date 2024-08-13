/**
 * @file s2e_math.hpp
 * @brief Math functions
 */

#ifndef S2E_LIBRARY_MATH_S2E_MATH_HPP_
#define S2E_LIBRARY_MATH_S2E_MATH_HPP_

#include <cmath>

namespace math {

/**
 * @fn WrapTo2Pi
 * @brief Wrap angle value into [0, 2pi]
 * @param angle_rad: Angle [rad]
 */
double WrapTo2Pi(const double angle_rad);

}  // namespace math

#endif  // S2E_LIBRARY_MATH_S2E_MATH_HPP_
