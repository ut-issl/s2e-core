/*
 * @file Antenna.cpp
 * @brief Component emulation: RF antenna
 */

#include "AntennaRadiationPattern.hpp"

#include <Interface/InitInput/IniAccess.h>

#include <Library/math/s2e_math.hpp>
#include <algorithm>

AntennaRadiationPattern::AntennaRadiationPattern() { gain_dBi_.assign(length_theta_, std::vector<double>(length_phi_, 0.0)); }

AntennaRadiationPattern::AntennaRadiationPattern(const std::string file_path, const size_t length_theta, const size_t length_phi,
                                                 const double theta_max_rad, const double phi_max_rad)
    : length_theta_(length_theta), length_phi_(length_phi), theta_max_rad_(theta_max_rad), phi_max_rad_(phi_max_rad) {
  IniAccess gain_file(file_path);
  gain_file.ReadCsvDouble(gain_dBi_, (std::max)(length_theta_, length_phi_));
}

AntennaRadiationPattern::~AntennaRadiationPattern() {}

double AntennaRadiationPattern::GetGain_dBi(const double theta_rad, const double phi_rad) const {
  // Argument check
  double theta_rad_clipped = theta_rad;
  double phi_rad_clipped = phi_rad;
  if (theta_rad_clipped < 0.0) theta_rad_clipped = 0.0;
  if (theta_rad_clipped > theta_max_rad_) theta_rad_clipped = theta_max_rad_;
  if (phi_rad_clipped < 0.0) phi_rad_clipped = 0.0;
  if (phi_rad_clipped > phi_max_rad_) phi_rad_clipped = phi_max_rad_;

  // Calc index
  size_t theta_idx = (size_t)(length_theta_ * theta_rad_clipped / theta_max_rad_ + 0.5);
  if (theta_idx >= length_theta_) theta_idx = length_theta_ - 1;
  size_t phi_idx = (size_t)(length_phi_ * phi_rad_clipped / phi_max_rad_ + 0.5);
  if (phi_idx >= length_phi_) phi_idx = length_phi_ - 1;

  return gain_dBi_[theta_idx][phi_idx];
}
