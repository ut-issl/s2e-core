/**
 * @file geomagnetic_field.cpp
 * @brief Class to calculate magnetic field of the earth
 */

#include "geomagnetic_field.hpp"

#include "math_physics/geomagnetic/igrf.h"
#include "math_physics/randomization/global_randomization.hpp"
#include "math_physics/randomization/normal_randomization.hpp"
#include "math_physics/randomization/random_walk.hpp"
#include "setting_file_reader/initialize_file_access.hpp"

GeomagneticField::GeomagneticField(const std::string igrf_file_name, const double random_walk_srandard_deviation_nT,
                                   const double random_walk_limit_nT, const double white_noise_standard_deviation_nT)
    : magnetic_field_i_nT_(0.0),
      magnetic_field_b_nT_(0.0),
      random_walk_standard_deviation_nT_(random_walk_srandard_deviation_nT),
      random_walk_limit_nT_(random_walk_limit_nT),
      white_noise_standard_deviation_nT_(white_noise_standard_deviation_nT),
      igrf_file_name_(igrf_file_name) {
  set_file_path(igrf_file_name_.c_str());
}

void GeomagneticField::CalcMagneticField(const double decimal_year, const double sidereal_day, const s2e::geodesy::GeodeticPosition position,
                                         const s2e::math::Quaternion quaternion_i2b) {
  if (!IsCalcEnabled) return;

  const double lat_rad = position.GetLatitude_rad();
  const double lon_rad = position.GetLongitude_rad();
  const double alt_m = position.GetAltitude_m();

  double magnetic_field_array_i_nT[3];
  IgrfCalc(decimal_year, lat_rad, lon_rad, alt_m, sidereal_day, magnetic_field_array_i_nT);
  AddNoise(magnetic_field_array_i_nT);
  for (int i = 0; i < 3; ++i) {
    magnetic_field_i_nT_[i] = magnetic_field_array_i_nT[i];
  }
  magnetic_field_b_nT_ = quaternion_i2b.FrameConversion(magnetic_field_i_nT_);
}

void GeomagneticField::AddNoise(double* magnetic_field_array_i_nT) {
  static s2e::math::Vector<3> standard_deviation(random_walk_standard_deviation_nT_);
  static s2e::math::Vector<3> limit(random_walk_limit_nT_);
  static RandomWalk<3> random_walk(0.1, standard_deviation, limit);

  static randomization::NormalRand white_noise(0.0, white_noise_standard_deviation_nT_, global_randomization.MakeSeed());

  for (int i = 0; i < 3; ++i) {
    magnetic_field_array_i_nT[i] += random_walk[i] + white_noise;
  }
  ++random_walk;  // Update random walk
}

std::string GeomagneticField::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("geomagnetic_field_at_spacecraft_position", "i", "nT", 3);
  str_tmp += WriteVector("geomagnetic_field_at_spacecraft_position", "b", "nT", 3);

  return str_tmp;
}

std::string GeomagneticField::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(magnetic_field_i_nT_);
  str_tmp += WriteVector(magnetic_field_b_nT_);

  return str_tmp;
}

GeomagneticField InitGeomagneticField(std::string initialize_file_path) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "MAGNETIC_FIELD_ENVIRONMENT";

  std::string fname = conf.ReadString(section, "coefficient_file");
  double mag_rwdev = conf.ReadDouble(section, "magnetic_field_random_walk_standard_deviation_nT");
  double mag_rwlimit = conf.ReadDouble(section, "magnetic_field_random_walk_limit_nT");
  double mag_wnvar = conf.ReadDouble(section, "magnetic_field_white_noise_standard_deviation_nT");

  GeomagneticField geomagnetic_field(fname, mag_rwdev, mag_rwlimit, mag_wnvar);
  geomagnetic_field.IsCalcEnabled = conf.ReadEnable(section, INI_CALC_LABEL);
  geomagnetic_field.is_log_enabled_ = conf.ReadEnable(section, INI_LOG_LABEL);

  return geomagnetic_field;
}
