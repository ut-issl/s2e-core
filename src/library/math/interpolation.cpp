/**
 * @file interpolation.cpp
 * @brief Implementation of mathematical interpolation method
 */

#include "interpolation.hpp"

#include <cmath>

namespace libra {

double Interpolation::CalcPolynomial(const double x) {
  // Search nearest point
  size_t nearest_x_id = FindNearestPoint(x);

  // Neville's algorithm
  double y_output = dependent_variables_[nearest_x_id];
  std::vector<double> down_diff = dependent_variables_;
  std::vector<double> up_diff = dependent_variables_;
  size_t d_idx = 1;
  for (size_t m = 1; m < degree_; m++) {
    // Calculate C and D
    for (size_t i = 0; i < degree_ - m; i++) {
      double denominator = independent_variables_[i] - independent_variables_[i + m];
      double down_minus_up = down_diff[i + 1] - up_diff[i];
      up_diff[i] = (independent_variables_[i + m] - x) * down_minus_up / denominator;
      down_diff[i] = (independent_variables_[i] - x) * down_minus_up / denominator;
    }

    // Upstream first calculation
    double dy;
    if (nearest_x_id >= m) {
      dy = up_diff[nearest_x_id - d_idx];
      d_idx++;
    } else {
      dy = down_diff[0];
    }
    y_output += dy;
  }

  return y_output;
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
