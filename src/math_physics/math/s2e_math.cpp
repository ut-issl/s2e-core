/**
 * @file s2e_math.cpp
 * @brief Math functions
 */

#include "s2e_math.hpp"

#include <math_physics/math/constants.hpp>

namespace math {
double WrapTo2Pi(const double angle_rad) {
  double angle_out = angle_rad;
  if (angle_out < 0.0) {
    while (angle_out < 0.0) {
      angle_out += math::tau;
    }
  } else if (angle_out > math::tau) {
    while (angle_out > math::tau) {
      angle_out -= math::tau;
    }
  } else {
    // nothing to do
  }
  return angle_out;
}
}  // namespace math
