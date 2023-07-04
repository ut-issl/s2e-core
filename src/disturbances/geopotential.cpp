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

#include "../library/logger/log_utility.hpp"
#include "../library/utilities/macros.hpp"

// #define DEBUG_GEOPOTENTIAL

Geopotential::Geopotential(const int degree, const std::string file_path, const bool is_calculation_enabled)
    : Disturbance(is_calculation_enabled, false), degree_(degree) {
  // Initialize
  acceleration_ecef_m_s2_ = libra::Vector<3>(0.0);
  debug_pos_ecef_m_ = libra::Vector<3>(0.0);
  // degree
  if (degree_ > 360) {
    degree_ = 360;
    std::cout << "Inputted degree of Geopotential is too large for EGM96 model(limit is 360)\n";
    std::cout << "degree of Geopotential set as " << degree_ << "\n";
  } else if (degree_ <= 1) {
    degree_ = 0;
  }
  // coefficients
  c_.assign(degree_ + 1, std::vector<double>(degree_ + 1, 0.0));
  s_.assign(degree_ + 1, std::vector<double>(degree_ + 1, 0.0));
  // For actual EGM model, c_[0][0] should be 1.0
  // In S2E, 0 degree term is inside the SimpleCircularOrbit calculation
  c_[0][0] = 0.0;
  if (degree_ >= 2) {
    if (!ReadCoefficientsEgm96(file_path)) {
      degree_ = 0;
      std::cout << "degree of Geopotential set as " << degree_ << "\n";
    }
  }
  // Initialize GravityPotential
  geopotential_ = new GravityPotential(degree, c_, s_);
}

bool Geopotential::ReadCoefficientsEgm96(std::string file_name) {
  std::ifstream coeff_file(file_name);
  if (!coeff_file.is_open()) {
    std::cerr << "File open error: Geopotential\n";
    return false;
  }

  size_t num_coeff = ((degree_ + 1) * (degree_ + 2) / 2) - 3;  //-3 for C00,C10,C11
  for (size_t i = 0; i < num_coeff; i++) {
    int n, m;
    double c_nm_norm, s_nm_norm;
    std::string line;
    getline(coeff_file, line);
    std::istringstream streamline(line);
    streamline >> n >> m >> c_nm_norm >> s_nm_norm;

    c_[n][m] = c_nm_norm;
    s_[n][m] = s_nm_norm;
  }
  return true;
}

void Geopotential::Update(const LocalEnvironment &local_environment, const Dynamics &dynamics) {
#ifdef DEBUG_GEOPOTENTIAL
  chrono::system_clock::time_point start, end;
  start = chrono::system_clock::now();
  debug_pos_ecef_m_ = spacecraft.dynamics_->orbit_->GetPosition_ecef_m();
#endif

  acceleration_ecef_m_s2_ = geopotential_->CalcAcceleration_xcxf_m_s2(dynamics.GetOrbit().GetPosition_ecef_m());
#ifdef DEBUG_GEOPOTENTIAL
  end = chrono::system_clock::now();
  time_ms_ = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
#else
  UNUSED(time_ms_);
#endif

  libra::Matrix<3, 3> trans_eci2ecef_ = local_environment.GetCelestialInformation().GetGlobalInformation().GetEarthRotation().GetDcmJ2000ToXcxf();
  libra::Matrix<3, 3> trans_ecef2eci = trans_eci2ecef_.Transpose();
  acceleration_i_m_s2_ = trans_ecef2eci * acceleration_ecef_m_s2_;
}

std::string Geopotential::GetLogHeader() const {
  std::string str_tmp = "";

#ifdef DEBUG_GEOPOTENTIAL
  str_tmp += WriteVector("geopotential_calculation_position_", "ecef", "m", 3);
  str_tmp += WriteScalar("geopotential_calculation_time", "ms");
#endif
  str_tmp += WriteVector("geopotential_acceleration", "ecef", "m/s2", 3);

  return str_tmp;
}

std::string Geopotential::GetLogValue() const {
  std::string str_tmp = "";

#ifdef DEBUG_GEOPOTENTIAL
  str_tmp += WriteVector(debug_pos_ecef_m_, 15);
  str_tmp += WriteScalar(time_ms_);
#endif

  str_tmp += WriteVector(acceleration_ecef_m_s2_, 15);

  return str_tmp;
}
