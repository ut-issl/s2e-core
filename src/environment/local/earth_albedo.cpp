/**
 * @file earth_albedo.cpp
 * @brief Class to manage earth albedo
 */
#include "earth_albedo.hpp"

#include <algorithm>
#include <cassert>
#include <fstream>

#include "library/initialize/initialize_file_access.hpp"
#include "library/logger/log_utility.hpp"
#include "library/math/constants.hpp"
#include "library/math/vector.hpp"

EarthAlbedo::EarthAlbedo() {}

std::string EarthAlbedo::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar("earth_albedo_factor");

  return str_tmp;
}

std::string EarthAlbedo::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(earth_albedo_factor_);

  return str_tmp;
}

EarthAlbedo InitEarthAlbedo(std::string initialize_file_path) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "EARTH_ALBEDO";

  EarthAlbedo earth_albedo;
  earth_albedo.IsCalcEarthAlbedoEnabled = conf.ReadEnable(section, INI_CALC_LABEL);
  earth_albedo.earth_albedo_factor_ = conf.ReadDouble(section, "earth_albedo_factor");

  return earth_albedo;
}
