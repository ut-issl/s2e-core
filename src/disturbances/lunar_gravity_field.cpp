/**
 * @file lunar_gravity_field.cpp
 * @brief Class to calculate the high-order lunar gravity acceleration
 */

#include "lunar_gravity_field.hpp"

#include <chrono>
#include <cmath>
#include <environment/global/physical_constants.hpp>
#include <fstream>
#include <iostream>
#include <library/initialize/initialize_file_access.hpp>

#include "../library/logger/log_utility.hpp"
#include "../library/utilities/macros.hpp"

#define DEBUG_LUNAR_GRAVITY_FIELD

LunarGravityField::LunarGravityField(const int degree, const std::string file_path, const bool is_calculation_enabled)
    : Disturbance(is_calculation_enabled, false), degree_(degree) {
  // Initialize
  acceleration_mcmf_m_s2_ = libra::Vector<3>(0.0);
  debug_pos_mcmf_m_ = libra::Vector<3>(0.0);
  debug_pos_mcmf_m_[0] = 2000000;
  debug_pos_mcmf_m_[1] = 2000000;
  debug_pos_mcmf_m_[2] = 2000000;
  // degree
  if (degree_ > 1200) {
    degree_ = 1200;
    std::cout << "Inputted degree of LunarGravityField is too large for GRGM1200A model(limit is 1200)\n";
    std::cout << "degree of LunarGravityField set as " << degree_ << "\n";
  } else if (degree_ <= 1) {
    degree_ = 0;
  }
  // coefficients
  c_.assign(degree_ + 1, std::vector<double>(degree_ + 1, 0.0));
  s_.assign(degree_ + 1, std::vector<double>(degree_ + 1, 0.0));
  // For actual GRGM model, c_[0][0] should be 1.0
  // In S2E, 0 degree term is inside the RK4 orbit calculation
  c_[0][0] = 0.0;
  if (degree_ >= 2) {
    if (!ReadCoefficientsGrgm1200a(file_path)) {
      degree_ = 0;
      std::cout << "degree of LunarGravityField set as " << degree_ << "\n";
    }
  }
  // Initialize GravityPotential
  lunar_potential_ = GravityPotential(degree, c_, s_, gravity_constants_km3_s2_ * 1e9, reference_radius_km_ * 1e3);
}

bool LunarGravityField::ReadCoefficientsGrgm1200a(std::string file_name) {
  std::ifstream coeff_file(file_name);
  if (!coeff_file.is_open()) {
    std::cerr << "File open error: LunarGravityField\n";
    return false;
  }

  // Read header
  std::string line, cell;
  getline(coeff_file, cell, ',');
  reference_radius_km_ = std::stod(cell);
  getline(coeff_file, cell, ',');
  gravity_constants_km3_s2_ = std::stod(cell);
  // next line
  getline(coeff_file, line);

  size_t num_coeff = ((degree_ + 1) * (degree_ + 2) / 2) - 1;  // -1 for C00
  for (size_t i = 0; i < num_coeff; i++) {
    // degree
    getline(coeff_file, line, ',');
    int n = std::stoi(line);
    getline(coeff_file, line, ',');
    int m = std::stoi(line);
    // coefficients
    getline(coeff_file, line, ',');
    double c_nm_norm = std::stod(line);
    getline(coeff_file, line, ',');
    double s_nm_norm = std::stod(line);
    // next line
    getline(coeff_file, line);

    c_[n][m] = c_nm_norm;
    s_[n][m] = s_nm_norm;
  }
  return true;
}

void LunarGravityField::Update(const LocalEnvironment &local_environment, const Dynamics &dynamics) {
  libra::Vector<3> position_mcmf_m = dynamics.GetOrbit().GetPosition_ecef_m();
#ifdef DEBUG_LUNAR_GRAVITY_FIELD
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();
  position_mcmf_m = debug_pos_mcmf_m_;
#endif

  acceleration_mcmf_m_s2_ = lunar_potential_.CalcAcceleration_xcxf_m_s2(position_mcmf_m);
#ifdef DEBUG_LUNAR_GRAVITY_FIELD
  end = std::chrono::system_clock::now();
  time_ms_ = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0);
#else
  UNUSED(time_ms_);
#endif

  libra::Matrix<3, 3> trans_eci2mcmf_ =
      local_environment.GetCelestialInformation().GetGlobalInformation().GetEarthRotation().GetDcmJ2000ToEcef();  // FIXME: use moon rotation
  libra::Matrix<3, 3> trans_mcmf2eci = trans_eci2mcmf_.Transpose();
  acceleration_i_m_s2_ = trans_mcmf2eci * acceleration_mcmf_m_s2_;
}

std::string LunarGravityField::GetLogHeader() const {
  std::string str_tmp = "";

#ifdef DEBUG_LUNAR_GRAVITY_FIELD
  str_tmp += WriteVector("lunar_gravity_calculation_position", "mcmf", "m", 3);
  str_tmp += WriteScalar("lunar_gravity_calculation_time", "ms");
#endif
  str_tmp += WriteVector("lunar_gravity_acceleration", "mcmf", "m/s2", 3);

  return str_tmp;
}

std::string LunarGravityField::GetLogValue() const {
  std::string str_tmp = "";

#ifdef DEBUG_LUNAR_GRAVITY_FIELD
  str_tmp += WriteVector(debug_pos_mcmf_m_, 15);
  str_tmp += WriteScalar(time_ms_);
#endif

  str_tmp += WriteVector(acceleration_mcmf_m_s2_, 15);

  return str_tmp;
}

LunarGravityField InitLunarGravityField(const std::string initialize_file_path) {
  auto conf = IniAccess(initialize_file_path);
  const char *section = "LUNAR_GRAVITY_FIELD";

  const int degree = conf.ReadInt(section, "degree");
  const std::string coefficients_file_path = conf.ReadString(section, "coefficients_file_path");

  const bool is_calc_enable = conf.ReadEnable(section, INI_CALC_LABEL);

  LunarGravityField lunar_gravity_field(degree, coefficients_file_path, is_calc_enable);
  lunar_gravity_field.is_log_enabled_ = conf.ReadEnable(section, INI_LOG_LABEL);

  return lunar_gravity_field;
}
