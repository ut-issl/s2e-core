/**
 * @file s2e_math.hpp
 * @brief Math functions
 */

#ifndef S2E_LIBRARY_MATH_S2E_MATH_H_
#define S2E_LIBRARY_MATH_S2E_MATH_H_

#include <cmath>

namespace libra {

/**
 * @fn WrapTo2Pi
 * @brief Wrap angle value into [0, 2pi]
 * @param angle: Angle [rad]
 */
double WrapTo2Pi(const double angle);

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_S2E_MATH_H_
