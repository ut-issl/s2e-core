/**
 * @file geopotential.cpp
 * @brief Class to calculate the high-order earth gravity acceleration
 */

#include "geopotential.hpp"

#include <chrono>
#include <cmath>
#include <environment/global/physical_constants.hpp>
#include <fstream>
#include <iostream>

#include "../interface/log_output/log_utility.hpp"

// #define DEBUG_GEOPOTENTIAL

GeoPotential::GeoPotential(const int degree, const std::string file_path, const bool is_calculation_enabled)
    : AccelerationDisturbance(is_calculation_enabled), degree_(degree) {
  // Initialize
  acc_ecef_ = libra::Vector<3>(0);
  debug_pos_ecef_m_ = libra::Vector<3>(0);
  // degree
  if (degree_ > 360) {
    degree_ = 360;
    std::cout << "Inputted degree of GeoPotential is too large for EGM96 "
                 "model(limit is 360)\n";
    std::cout << "degree of GeoPotential set as " << degree_ << "\n";
  } else if (degree_ <= 1) {
    degree_ = 0;
  }
  // coefficients
  c_.assign(degree_ + 1, vector<double>(degree_ + 1, 0.0));
  s_.assign(degree_ + 1, vector<double>(degree_ + 1, 0.0));
  // For actual EGM model, c_[0][0] should be 1.0
  // In S2E, 0 degree term is inside the SimpleCircularOrbit calculation
  c_[0][0] = 0.0;
  if (degree_ >= 2) {
    if (!ReadCoefficientsEGM96(file_path)) {
      degree_ = 0;
      std::cout << "degree of GeoPotential set as " << degree_ << "\n";
    }
  }
}

bool GeoPotential::ReadCoefficientsEGM96(std::string file_name) {
  std::ifstream coeff_file(file_name);
  if (!coeff_file.is_open()) {
    std::cerr << "file open error:Geopotential\n";
    return false;
  }

  int num_coeff = ((degree_ + 1) * (degree_ + 2) / 2) - 3;  //-3 for C00,C10,C11
  for (int i = 0; i < num_coeff; i++) {
    int n_, m_;
    double c_nm_norm, s_nm_norm;
    std::string line;
    getline(coeff_file, line);
    std::istringstream streamline(line);
    streamline >> n_ >> m_ >> c_nm_norm >> s_nm_norm;

    c_[n_][m_] = c_nm_norm;
    s_[n_][m_] = s_nm_norm;
  }
  return true;
}

void GeoPotential::Update(const LocalEnvironment &local_env, const Dynamics &dynamics) {
#ifdef DEBUG_GEOPOTENTIAL
  chrono::system_clock::time_point start, end;
  start = chrono::system_clock::now();
  debug_pos_ecef_m_ = spacecraft.dynamics_->orbit_->GetSatPosition_ecef();
#endif

  CalcAccelerationECEF(dynamics.GetOrbit().GetSatPosition_ecef());
#ifdef DEBUG_GEOPOTENTIAL
  end = chrono::system_clock::now();
  time_ = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
#endif

  Matrix<3, 3> trans_eci2ecef_ = local_env.GetCelesInfo().GetGlobalInfo().GetEarthRotation().GetDCMJ2000toXCXF();
  Matrix<3, 3> trans_ecef2eci = transpose(trans_eci2ecef_);
  acceleration_i_m_s2_ = trans_ecef2eci * acc_ecef_;
}

void GeoPotential::CalcAccelerationECEF(const Vector<3> &position_ecef) {
  x = position_ecef[0], y = position_ecef[1], z = position_ecef[2];
  r = sqrt(x * x + y * y + z * z);

  // Calc V and W
  int degree_vw = degree_ + 1;
  vector<vector<double>> v(degree_vw + 1, vector<double>(degree_vw + 1, 0.0));
  vector<vector<double>> w(degree_vw + 1, vector<double>(degree_vw + 1, 0.0));
  // n=m=0
  v[0][0] = environment::earth_equatorial_radius_m / r;
  w[0][0] = 0.0;
  m = 0;

  while (m < degree_vw) {
    for (n = m + 1; n <= degree_vw; n++) {
      if (n <= m + 1)
        v_w_nm_update(&v[n][m], &w[n][m], v[n - 1][m], w[n - 1][m], 0.0, 0.0);
      else
        v_w_nm_update(&v[n][m], &w[n][m], v[n - 1][m], w[n - 1][m], v[n - 2][m], w[n - 2][m]);
    }
    // next step
    m++;
    n = m;
    v_w_nn_update(&v[n][m], &w[n][m], v[n - 1][m - 1], w[n - 1][m - 1]);
  }

  // Calc Acceleration
  acc_ecef_ *= 0.0;
  for (n = 0; n <= degree_; n++)  // this loop can integrate with previous loop
  {
    m = 0;
    double n_d = (double)n;
    double normalize = sqrt((2.0 * n_d + 1.0) / (2.0 * n_d + 3.0));
    double normalize_xy = normalize * sqrt((n_d + 2.0) * (n_d + 1.0) / 2.0);
    // m==0
    acc_ecef_[0] += -c_[n][0] * v[n + 1][1] * normalize_xy;
    acc_ecef_[1] += -c_[n][0] * w[n + 1][1] * normalize_xy;
    acc_ecef_[2] += (n + 1.0) * (-c_[n][0] * v[n + 1][0] - s_[n][0] * w[n + 1][0]) * normalize;
    for (m = 1; m <= n; m++) {
      double m_d = (double)m;
      double factorial = (n_d - m_d + 1.0) * (n_d - m_d + 2.0);
      double normalize_xy1 = normalize * sqrt((n_d + m_d + 1.0) * (n_d + m_d + 2.0));
      double normalize_xy2;
      if (m == 1)
        normalize_xy2 = normalize * sqrt(factorial) * sqrt(2.0);
      else
        normalize_xy2 = normalize * sqrt(factorial);
      double normalize_z = normalize * sqrt((n_d + m_d + 1.0) / (n_d - m_d + 1.0));

      acc_ecef_[0] += 0.5 * (normalize_xy1 * (-c_[n][m] * v[n + 1][m + 1] - s_[n][m] * w[n + 1][m + 1]) +
                             normalize_xy2 * (c_[n][m] * v[n + 1][m - 1] + s_[n][m] * w[n + 1][m - 1]));
      acc_ecef_[1] += 0.5 * (normalize_xy1 * (-c_[n][m] * w[n + 1][m + 1] + s_[n][m] * v[n + 1][m + 1]) +
                             normalize_xy2 * (-c_[n][m] * w[n + 1][m - 1] + s_[n][m] * v[n + 1][m - 1]));
      acc_ecef_[2] += (n_d - m_d + 1.0) * (-c_[n][m] * v[n + 1][m] - s_[n][m] * w[n + 1][m]) * normalize_z;
    }
  }
  acc_ecef_ *= environment::earth_gravitational_constant_m3_s2 / (environment::earth_equatorial_radius_m * environment::earth_equatorial_radius_m);

  return;
}

void GeoPotential::v_w_nn_update(double *v_nn, double *w_nn, const double v_prev, const double w_prev) {
  if (n != m) return;

  double n_d = (double)n;

  double tmp = environment::earth_equatorial_radius_m / (r * r);
  double x_tmp = x * tmp;
  double y_tmp = y * tmp;
  double c_normalize;
  if (n == 1)
    c_normalize = (2.0 * n_d - 1.0) * sqrt(2.0 * n_d + 1.0);
  else
    c_normalize = sqrt((2.0 * n_d + 1.0) / (2.0 * n_d));

  *v_nn = c_normalize * (x_tmp * v_prev - y_tmp * w_prev);
  *w_nn = c_normalize * (x_tmp * w_prev + y_tmp * v_prev);
  return;
}

void GeoPotential::v_w_nm_update(double *v_nm, double *w_nm, const double v_prev, const double w_prev, const double v_prev2, const double w_prev2) {
  if (n == m) return;

  double m_d = (double)m;
  double n_d = (double)n;

  double tmp = environment::earth_equatorial_radius_m / (r * r);
  double z_tmp = z * tmp;
  double re_tmp = environment::earth_equatorial_radius_m * tmp;
  double c1 = (2.0 * n_d - 1.0) / (n_d - m_d);
  double c2 = (n_d + m_d - 1.0) / (n_d - m_d);
  double c_normalize, c2_normalize;

  c_normalize = sqrt(((2.0 * n_d + 1.0) * (n_d - m_d)) / ((2.0 * n_d - 1.0) * (n_d + m_d)));
  if (n <= 1)
    c2_normalize = 1.0;
  else
    c2_normalize = sqrt(((2.0 * n_d - 1.0) * (n_d - m_d - 1.0)) / ((2.0 * n_d - 3.0) * (n_d + m_d - 1.0)));

  *v_nm = c_normalize * (c1 * z_tmp * v_prev - c2 * c2_normalize * re_tmp * v_prev2);
  *w_nm = c_normalize * (c1 * z_tmp * w_prev - c2 * c2_normalize * re_tmp * w_prev2);
  return;
}

std::string GeoPotential::GetLogHeader() const {
  std::string str_tmp = "";

#ifdef DEBUG_GEOPOTENTIAL
  str_tmp += WriteVector("pos_", "ecef", "m", 3);
  str_tmp += WriteScalar("time_geop", "ms");
#endif
  str_tmp += WriteVector("geopotential_acceleration", "ecef", "m/s2", 3);

  return str_tmp;
}

std::string GeoPotential::GetLogValue() const {
  std::string str_tmp = "";

#ifdef DEBUG_GEOPOTENTIAL
  str_tmp += WriteVector(debug_pos_ecef_m_, 15);
  str_tmp += WriteScalar(time_);
#endif

  str_tmp += WriteVector(acc_ecef_, 15);

  return str_tmp;
}
