/**
 * @file interpolation.cpp
 * @brief Implementation of mathematical interpolation method
 */

#include "interpolation.hpp"

#include <cmath>

namespace math

double Interpolation::CalcPolynomial(const double x) const {
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

double Interpolation::CalcTrigonometric(const double x, const double period) const {
  double y_output = 0.0;
  size_t end_id = degree_;
  size_t start_id = 0;

  // Modify to odd number degrees
  // TODO: implement more efficient method
  if (degree_ % 2 == 0) {
    size_t nearest_point = FindNearestPoint(x);
    // Remove the farthest point
    if (nearest_point * 2 < degree_) {
      end_id--;
    } else {
      start_id++;
    }
  }

  for (size_t i = start_id; i < end_id; ++i) {
    double t_k = 1.0;
    for (size_t j = start_id; j < end_id; ++j) {
      if (i == j) continue;
      t_k *= sin(period * (x - independent_variables_[j]) / 2.0) / sin(period * (independent_variables_[i] - independent_variables_[j]) / 2.0);
    }
    y_output += t_k * dependent_variables_[i];
  }

  return y_output;
}

bool Interpolation::PushAndPopData(const double independent_variable, const double dependent_variable) {
  if (independent_variable <= independent_variables_.back()) {
    return false;
  }
  independent_variables_.erase(independent_variables_.begin());
  independent_variables_.push_back(independent_variable);
  dependent_variables_.erase(dependent_variables_.begin());
  dependent_variables_.push_back(dependent_variable);
  return true;
}

size_t Interpolation::FindNearestPoint(const double x) const {
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
