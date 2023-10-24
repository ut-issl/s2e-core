/**
 * @file interpolation_implementation.hpp
 * @brief Implementation of mathematical interpolation method
 */

#include "interpolation.hpp"

#include <cmath>

namespace libra {

double Interpolation::CalcPolynomial(const double x) {
  std::vector<double> c = dependent_variables_;
  std::vector<double> d = dependent_variables_;

  // Search nearest point
  size_t nearest_x_id = FindNearestPoint(x);

  //
  double result = dependent_variables_[nearest_x_id];
  for (size_t m = 1; m < degree_; m++) {
    for (size_t i = 0; i < degree_ - m; i++) {
      double ho = independent_variables_[i] - x;
      double hp = independent_variables_[i + m] - x;
      double den = ho - hp;
      double w = c[i + 1] - d[i];
      den = w / den;
      d[i] = hp * den;
      c[i] = ho * den;
    }
    double dy;
    size_t d_idx = 1;
    if (nearest_x_id >= m) {
      dy = d[nearest_x_id - d_idx];
      d_idx++;
    } else {
      dy = c[0];
    }
    result += dy;
  }

  return result;
}

size_t Interpolation::FindNearestPoint(const double x) {
  size_t output = 0;
  double difference1 = fabs(x - independent_variables_[0]);
  for (size_t i = 0; i < degree_; i++) {
    double difference2 = fabs(x - independent_variables_[i]);
    if (difference2 < difference1) {
      difference1 = difference2;
      output = i;
    }
  }
  return output;
}

}  // namespace libra
