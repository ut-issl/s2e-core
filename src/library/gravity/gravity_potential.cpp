/**
 * @file gravity_potential.cpp
 * @brief Class to calculate gravity potential
 */

#include "gravity_potential.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>

#include "../library/logger/log_utility.hpp"
#include "../library/utilities/macros.hpp"

GravityPotential::GravityPotential(const size_t degree, const std::vector<std::vector<double>> cosine_coefficients,
                                   const std::vector<std::vector<double>> sine_coefficients, const double gravity_constants_m3_s2,
                                   const double center_body_radius_m)
    : degree_(degree),
      c_(cosine_coefficients),
      s_(sine_coefficients),
      gravity_constants_m3_s2_(gravity_constants_m3_s2),
      center_body_radius_m_(center_body_radius_m) {
  // degree
  if (degree_ <= 1) {
    degree_ = 0;
  }
  // coefficients
  // TODO Check size
}

libra::Vector<3> GravityPotential::CalcAcceleration_xcxf_m_s2(const libra::Vector<3> &position_xcxf_m) {
  libra::Vector<3> acceleration_xcxf_m_s2(0.0);
  if (degree_ <= 0) return acceleration_xcxf_m_s2;

  xcxf_x_m_ = position_xcxf_m[0];
  xcxf_y_m_ = position_xcxf_m[1];
  xcxf_z_m_ = position_xcxf_m[2];
  radius_m_ = position_xcxf_m.CalcNorm();

  // Calc V and W
  const size_t degree_vw = degree_ + 1;
  std::vector<std::vector<double>> v(degree_vw + 1, std::vector<double>(degree_vw + 1, 0.0));
  std::vector<std::vector<double>> w(degree_vw + 1, std::vector<double>(degree_vw + 1, 0.0));
  // n = m = 0
  v[0][0] = center_body_radius_m_ / radius_m_;
  w[0][0] = 0.0;
  m_ = 0;

  while (m_ < degree_vw) {
    for (n_ = m_ + 1; n_ <= degree_vw; n_++) {
      if (n_ <= m_ + 1) {
        v_w_nm_update(&v[n_][m_], &w[n_][m_], v[n_ - 1][m_], w[n_ - 1][m_], 0.0, 0.0);
      } else {
        v_w_nm_update(&v[n_][m_], &w[n_][m_], v[n_ - 1][m_], w[n_ - 1][m_], v[n_ - 2][m_], w[n_ - 2][m_]);
      }
    }
    // next step
    m_++;
    n_ = m_;
    v_w_nn_update(&v[n_][m_], &w[n_][m_], v[n_ - 1][m_ - 1], w[n_ - 1][m_ - 1]);
  }

  // Calc Acceleration
  for (n_ = 0; n_ <= degree_; n_++)  // this loop can integrate with previous loop
  {
    m_ = 0;
    const double n_d = (double)n_;
    const double normalize = sqrt((2.0 * n_d + 1.0) / (2.0 * n_d + 3.0));
    const double normalize_xy = normalize * sqrt((n_d + 2.0) * (n_d + 1.0) / 2.0);
    // m_==0
    acceleration_xcxf_m_s2[0] += -c_[n_][0] * v[n_ + 1][1] * normalize_xy;
    acceleration_xcxf_m_s2[1] += -c_[n_][0] * w[n_ + 1][1] * normalize_xy;
    acceleration_xcxf_m_s2[2] += (n_ + 1.0) * (-c_[n_][0] * v[n_ + 1][0] - s_[n_][0] * w[n_ + 1][0]) * normalize;
    for (m_ = 1; m_ <= n_; m_++) {
      const double m_d = (double)m_;
      const double factorial = (n_d - m_d + 1.0) * (n_d - m_d + 2.0);
      const double normalize_xy1 = normalize * sqrt((n_d + m_d + 1.0) * (n_d + m_d + 2.0));
      double normalize_xy2;
      if (m_ == 1) {
        normalize_xy2 = normalize * sqrt(factorial) * sqrt(2.0);
      } else {
        normalize_xy2 = normalize * sqrt(factorial);
      }
      const double normalize_z = normalize * sqrt((n_d + m_d + 1.0) / (n_d - m_d + 1.0));

      acceleration_xcxf_m_s2[0] += 0.5 * (normalize_xy1 * (-c_[n_][m_] * v[n_ + 1][m_ + 1] - s_[n_][m_] * w[n_ + 1][m_ + 1]) +
                                          normalize_xy2 * (c_[n_][m_] * v[n_ + 1][m_ - 1] + s_[n_][m_] * w[n_ + 1][m_ - 1]));
      acceleration_xcxf_m_s2[1] += 0.5 * (normalize_xy1 * (-c_[n_][m_] * w[n_ + 1][m_ + 1] + s_[n_][m_] * v[n_ + 1][m_ + 1]) +
                                          normalize_xy2 * (-c_[n_][m_] * w[n_ + 1][m_ - 1] + s_[n_][m_] * v[n_ + 1][m_ - 1]));
      acceleration_xcxf_m_s2[2] += (n_d - m_d + 1.0) * (-c_[n_][m_] * v[n_ + 1][m_] - s_[n_][m_] * w[n_ + 1][m_]) * normalize_z;
    }
  }
  acceleration_xcxf_m_s2 *= gravity_constants_m3_s2_ / pow(center_body_radius_m_, 2.0);

  return acceleration_xcxf_m_s2;
}

libra::Matrix<3, 3> GravityPotential::CalcPartialDerivative_xcxf_s2(const libra::Vector<3> &position_xcxf_m) {
  libra::Matrix<3, 3> partial_derivative{0.0};

  return partial_derivative;
}

void GravityPotential::v_w_nn_update(double *v_nn, double *w_nn, const double v_prev, const double w_prev) {
  if (n_ != m_) return;

  const double n_d = (double)n_;

  const double tmp = center_body_radius_m_ / pow(radius_m_, 2.0);
  const double x_tmp = xcxf_x_m_ * tmp;
  const double y_tmp = xcxf_y_m_ * tmp;
  double c_normalize;
  if (n_ == 1) {
    c_normalize = (2.0 * n_d - 1.0) * sqrt(2.0 * n_d + 1.0);
  } else {
    c_normalize = sqrt((2.0 * n_d + 1.0) / (2.0 * n_d));
  }

  *v_nn = c_normalize * (x_tmp * v_prev - y_tmp * w_prev);
  *w_nn = c_normalize * (x_tmp * w_prev + y_tmp * v_prev);
  return;
}

void GravityPotential::v_w_nm_update(double *v_nm, double *w_nm, const double v_prev, const double w_prev, const double v_prev2,
                                     const double w_prev2) {
  if (n_ == m_) return;

  const double m_d = (double)m_;
  const double n_d = (double)n_;

  const double tmp = center_body_radius_m_ / pow(radius_m_, 2.0);
  const double z_tmp = xcxf_z_m_ * tmp;
  const double re_tmp = center_body_radius_m_ * tmp;
  const double c1 = (2.0 * n_d - 1.0) / (n_d - m_d);
  const double c2 = (n_d + m_d - 1.0) / (n_d - m_d);
  double c_normalize, c2_normalize;

  c_normalize = sqrt(((2.0 * n_d + 1.0) * (n_d - m_d)) / ((2.0 * n_d - 1.0) * (n_d + m_d)));
  if (n_ <= 1) {
    c2_normalize = 1.0;
  } else {
    c2_normalize = sqrt(((2.0 * n_d - 1.0) * (n_d - m_d - 1.0)) / ((2.0 * n_d - 3.0) * (n_d + m_d - 1.0)));
  }

  *v_nm = c_normalize * (c1 * z_tmp * v_prev - c2 * c2_normalize * re_tmp * v_prev2);
  *w_nm = c_normalize * (c1 * z_tmp * w_prev - c2 * c2_normalize * re_tmp * w_prev2);
  return;
}
