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
    : magnetic_field_i_nT_(0.0),
      magnetic_field_b_nT_(0.0),
      random_walk_standard_deviation_nT_(random_walk_srandard_deviation_nT),
      random_walk_limit_nT_(random_walk_limit_nT),
      white_noise_standard_deviation_nT_(white_noise_standard_deviation_nT),
      igrf_file_name_(igrf_file_name) {
  set_file_path(igrf_file_name_.c_str());
}

void MagEnvironment::CalcMag(double decimal_year, double sidereal_day, Vector<3> lat_lon_alt, Quaternion quaternion_i2b) {
  if (!IsCalcEnabled) return;

  double lat_rad = lat_lon_alt(0);
  double lon_rad = lat_lon_alt(1);
  double alt_m = lat_lon_alt(2);

  double magnetic_field_array_i_nT[3];
  IgrfCalc(decimal_year, lat_rad, lon_rad, alt_m, sidereal_day, magnetic_field_array_i_nT);
  AddNoise(magnetic_field_array_i_nT);
  for (int i = 0; i < 3; ++i) {
    magnetic_field_i_nT_[i] = magnetic_field_array_i_nT[i];
  }
  magnetic_field_b_nT_ = quaternion_i2b.frame_conv(magnetic_field_i_nT_);
}

void MagEnvironment::AddNoise(double* magnetic_field_array_i_nT) {
  static Vector<3> standard_deviation(random_walk_standard_deviation_nT_);
  static Vector<3> limit(random_walk_limit_nT_);
  static RandomWalk<3> random_walk(0.1, standard_deviation, limit);

  static libra::NormalRand white_noise(0.0, white_noise_standard_deviation_nT_, g_rand.MakeSeed());

  for (int i = 0; i < 3; ++i) {
    magnetic_field_array_i_nT[i] += random_walk[i] + white_noise;
  }
  ++random_walk;  // Update random walk
}

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
