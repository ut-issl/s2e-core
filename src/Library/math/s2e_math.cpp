/**
 * @file s2e_math.cpp
 * @brief Math functions
 */

#include "s2e_math.hpp"

#include <Library/math/Constant.hpp>

namespace libra {
double WrapTo2Pi(const double angle) {
  double angle_out = angle;
  if (angle_out < 0.0) {
    while (angle_out < 0.0) {
      angle_out += libra::tau;
    }
  } else if (angle_out > libra::tau) {
    while (angle_out > libra::tau) {
      angle_out -= libra::tau;
    }
  } else {
    // nothing to do
  }
  return angle_out;
}
}  // namespace libra
