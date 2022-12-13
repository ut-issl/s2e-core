/**
 * @file s2e_math.hpp
 * @brief Math functions
 */

#pragma once
#include <cmath>

namespace libra {

/**
 * @fn WrapTo2Pi
 * @brief Wrap angle value into [0, 2pi]
 * @param angle: Angle [rad]
 */
double WrapTo2Pi(const double angle);

}  // namespace libra
