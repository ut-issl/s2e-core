/**
 * @file earth_albedo.cpp
 * @brief Class to manage earth albedo
 */
#include "earth_albedo.hpp"

#include <algorithm>
#include <cassert>
#include <fstream>

#include "logger/log_utility.hpp"
#include "math_physics/math/constants.hpp"
#include "math_physics/math/vector.hpp"
#include "setting_file_reader/initialize_file_access.hpp"

namespace s2e::environment {

EarthAlbedo::EarthAlbedo(LocalCelestialInformation* local_celestial_information, SolarRadiationPressureEnvironment* srp_environment)
    : local_celestial_information_(local_celestial_information), srp_environment_(srp_environment) {}

void EarthAlbedo::UpdateAllStates() {
  if (!GetIsCalcEarthAlbedoEnabled()) return;
  CalcEarthAlbedo(local_celestial_information_);
}

std::string EarthAlbedo::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar("earth_albedo_factor");
  str_tmp += WriteScalar("earth_albedo_W_m2");

  return str_tmp;
}

std::string EarthAlbedo::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(earth_albedo_factor_);
  str_tmp += WriteScalar(earth_albedo_W_m2_);

  return str_tmp;
}

void EarthAlbedo::CalcEarthAlbedo(const LocalCelestialInformation* local_celestial_information) {
  s2e::math::Vector<3> earth_position_b_m = local_celestial_information->GetPositionFromSpacecraft_b_m("EARTH");
  double earth_distance_m = earth_position_b_m.CalcNorm();
  earth_albedo_W_m2_ = srp_environment_->GetPowerDensity_W_m2() * GetEarthAlbedoFactor() *
                       pow((environment::astronomy::earth_equatorial_radius_m / earth_distance_m), 2.0) / 4.0;
}

EarthAlbedo InitEarthAlbedo(std::string initialize_file_path, LocalCelestialInformation* local_celestial_information,
                            SolarRadiationPressureEnvironment* srp_environment) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "EARTH_ALBEDO";

  EarthAlbedo earth_albedo(local_celestial_information, srp_environment);
  earth_albedo.SetIsCalcEarthAlbedoEnabled(conf.ReadEnable(section, INI_CALC_LABEL));
  earth_albedo.SetEarthAlbedoFactor(conf.ReadDouble(section, "earth_albedo_factor"));

  return earth_albedo;
}

} // namespace s2e::environment
