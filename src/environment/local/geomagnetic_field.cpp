/**
 * @file geomagnetic_field.cpp
 * @brief Class to calculate magnetic field of the earth
 */

#include "geomagnetic_field.hpp"

#include "library/external/igrf/igrf.h"
#include "library/initialize/initialize_file_access.hpp"
#include "library/randomization/global_randomization.hpp"
#include "library/randomization/normal_randomization.hpp"
#include "library/randomization/random_walk.hpp"

MagEnvironment::MagEnvironment(std::string igrf_file_name, double random_walk_srandard_deviation_nT, double random_walk_limit_nT,
                               double white_noise_standard_deviation_nT)
    : random_walk_srandard_deviation_nT_(random_walk_srandard_deviation_nT),
      random_walk_limit_nT_(random_walk_limit_nT),
      white_noise_standard_deviation_nT_(white_noise_standard_deviation_nT),
      igrf_file_name_(igrf_file_name) {
  for (int i = 0; i < 3; ++i) {
    magnetic_field_i_nT_[i] = 0;
  }
  for (int i = 0; i < 3; ++i) {
    magnetic_field_b_nT_[i] = 0;
  }
  set_file_path(igrf_file_name_.c_str());
}

void MagEnvironment::CalcMag(double decyear, double side, Vector<3> lat_lon_alt, Quaternion q_i2b) {
  if (!IsCalcEnabled) return;

  double latrad = lat_lon_alt(0);
  double lonrad = lat_lon_alt(1);
  double alt = lat_lon_alt(2);

  double mag_i_array[3];
  IgrfCalc(decyear, latrad, lonrad, alt, side, mag_i_array);
  AddNoise(mag_i_array);
  for (int i = 0; i < 3; ++i) {
    magnetic_field_i_nT_[i] = mag_i_array[i];
  }
  magnetic_field_b_nT_ = q_i2b.frame_conv(magnetic_field_i_nT_);
}

void MagEnvironment::AddNoise(double* mag_i_array) {
  static Vector<3> stddev(random_walk_srandard_deviation_nT_);
  static Vector<3> limit(random_walk_limit_nT_);
  static RandomWalk<3> rw(0.1, stddev, limit);
  static libra::NormalRand nr(0.0, white_noise_standard_deviation_nT_, g_rand.MakeSeed());
  for (int i = 0; i < 3; ++i) {
    mag_i_array[i] += rw[i] + nr;
  }
  ++rw;  // Update random walk
}

Vector<3> MagEnvironment::GetMag_i() const { return magnetic_field_i_nT_; }

Vector<3> MagEnvironment::GetMag_b() const { return magnetic_field_b_nT_; }

std::string MagEnvironment::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("geomagnetic_field_at_spacecraft_position", "i", "nT", 3);
  str_tmp += WriteVector("geomagnetic_field_at_spacecraft_position", "b", "nT", 3);

  return str_tmp;
}

std::string MagEnvironment::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(magnetic_field_i_nT_);
  str_tmp += WriteVector(magnetic_field_b_nT_);

  return str_tmp;
}
