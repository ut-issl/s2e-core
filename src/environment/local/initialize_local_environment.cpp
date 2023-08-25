/**
 * @file initialize_local_environment.cpp
 * @brief Initialize functions for local environment classes
 */

#include "initialize_local_environment.hpp"

#include <library/initialize/initialize_file_access.hpp>
#include <string>

#define CALC_LABEL "calculation"
#define LOG_LABEL "logging"

GeomagneticField InitGeomagneticField(std::string initialize_file_path) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "MAGNETIC_FIELD_ENVIRONMENT";

  std::string fname = conf.ReadString(section, "coefficient_file");
  double mag_rwdev = conf.ReadDouble(section, "magnetic_field_random_walk_standard_deviation_nT");
  double mag_rwlimit = conf.ReadDouble(section, "magnetic_field_random_walk_limit_nT");
  double mag_wnvar = conf.ReadDouble(section, "magnetic_field_white_noise_standard_deviation_nT");

  GeomagneticField geomagnetic_field(fname, mag_rwdev, mag_rwlimit, mag_wnvar);
  geomagnetic_field.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  geomagnetic_field.is_log_enabled_ = conf.ReadEnable(section, LOG_LABEL);

  return geomagnetic_field;
}

SolarRadiationPressureEnvironment InitSolarRadiationPressureEnvironment(std::string initialize_file_path,
                                                                        LocalCelestialInformation* local_celestial_information) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "SOLAR_RADIATION_PRESSURE_ENVIRONMENT";

  SolarRadiationPressureEnvironment srp_env(local_celestial_information);
  srp_env.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  srp_env.is_log_enabled_ = conf.ReadEnable(section, LOG_LABEL);

  return srp_env;
}
