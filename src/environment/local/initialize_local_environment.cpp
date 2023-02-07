/**
 * @file initialize_local_environment.cpp
 * @brief Initialize functions for local environment classes
 */

#include "initialize_local_environment.hpp"

#include <Interface/InitInput/IniAccess.h>

#include <string>

#define CALC_LABEL "calculation"
#define LOG_LABEL "logging"

MagEnvironment InitMagEnvironment(std::string ini_path) {
  auto conf = IniAccess(ini_path);
  const char* section = "MAGNETIC_FIELD_ENVIRONMENT";

  std::string fname = conf.ReadString(section, "coefficient_file");
  double mag_rwdev = conf.ReadDouble(section, "magnetic_field_random_walk_speed_nT");
  double mag_rwlimit = conf.ReadDouble(section, "magnetic_field_random_walk_limit_nT");
  double mag_wnvar = conf.ReadDouble(section, "magnetic_field_white_noise_standard_deviation_nT");

  MagEnvironment mag_env(fname, mag_rwdev, mag_rwlimit, mag_wnvar);
  mag_env.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  mag_env.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return mag_env;
}

SRPEnvironment InitSRPEnvironment(std::string ini_path, LocalCelestialInformation* local_celes_info) {
  auto conf = IniAccess(ini_path);
  const char* section = "SOLAR_RADIATION_PRESSURE_ENVIRONMENT";

  SRPEnvironment srp_env(local_celes_info);
  srp_env.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  srp_env.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return srp_env;
}

Atmosphere InitAtmosphere(std::string ini_path) {
  auto conf = IniAccess(ini_path);
  const char* section = "ATMOSPHERE";
  double f107_threshold = 50.0;
  double f107_default = 150.0;

  std::string model = conf.ReadString(section, "model");
  std::string table_path = conf.ReadString(section, "nrlmsise00_table_file");
  double rho_stddev = conf.ReadDouble(section, "air_density_standard_deviation");
  bool is_manual_param_used = conf.ReadEnable(section, "is_manual_parameter_used");
  double manual_daily_f107 = conf.ReadDouble(section, "manual_daily_f107");
  if (manual_daily_f107 < f107_threshold) {
    std::cerr << "Daily F10.7 may be too low. It is set as 150.0 in this simulation. "
                 "Check [ATMOSPHERE] section in LocalEnvironment.ini."
              << std::endl;
    manual_daily_f107 = f107_default;
  }
  double manual_average_f107 = conf.ReadDouble(section, "manual_average_f107");
  if (manual_average_f107 < f107_threshold) {
    std::cerr << "Average F10.7 may be too low. It is set as 150.0 in this "
                 "simulation. Check [ATMOSPHERE] section in LocalEnvironment.ini."
              << std::endl;
    manual_average_f107 = f107_default;
  }
  double manual_ap = conf.ReadDouble(section, "manual_ap");

  Atmosphere atmosphere(model, table_path, rho_stddev, is_manual_param_used, manual_daily_f107, manual_average_f107, manual_ap);
  atmosphere.IsCalcEnabled = conf.ReadEnable(section, CALC_LABEL);
  atmosphere.IsLogEnabled = conf.ReadEnable(section, LOG_LABEL);

  return atmosphere;
}
