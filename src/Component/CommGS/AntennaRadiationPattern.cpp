/*
 * @file Antenna.cpp
 * @brief Component emulation: RF antenna
 */

#include "AntennaRadiationPattern.hpp"

#include <Interface/InitInput/IniAccess.h>

#include <Library/math/s2e_math.hpp>

AntennaRadiationPattern::AntennaRadiationPattern() { gain_dB_.assign(length_theta_, std::vector<double>(length_phi_, 0.0)); }

AntennaRadiationPattern::AntennaRadiationPattern(const std::string file_path) {
  IniAccess gain_file(file_path);
  gain_file.ReadCsvDouble(gain_dB_, std::max(length_theta_, length_phi_));
}

AntennaRadiationPattern::~AntennaRadiationPattern() {}

double AntennaRadiationPattern::GetGain_dB(const double theta_rad, const double phi_rad) {
  // Argument check
  double theta_rad_clipped = theta_rad;
  double phi_rad_clipped = phi_rad;
  if (theta_rad_clipped < 0.0) theta_rad_clipped = 0.0;
  if (theta_rad_clipped > theta_max_rad_) theta_rad_clipped = theta_max_rad_;
  if (phi_rad_clipped < 0.0) phi_rad_clipped = 0.0;
  if (phi_rad_clipped > phi_max_rad_) phi_rad_clipped = phi_max_rad_;

  // Calc index
  size_t theta_idx = (size_t)(length_theta_ * theta_rad_clipped / theta_max_rad_);
  if (theta_idx > length_theta_) theta_idx = length_theta_;
  size_t phi_idx = (size_t)(length_phi_ * phi_rad_clipped / phi_max_rad_);
  if (phi_idx > length_phi_) phi_idx = length_phi_;

  return gain_dB_[theta_idx][phi_idx];
}
