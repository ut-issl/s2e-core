/**
 * @file earth_infrared.cpp
 * @brief Class to manage earth infrared
 */
#include "earth_infrared.hpp"

#include <algorithm>
#include <cassert>
#include <fstream>

#include "logger/log_utility.hpp"
#include "math_physics/math/constants.hpp"
#include "math_physics/math/vector.hpp"
#include "setting_file_reader/initialize_file_access.hpp"

namespace s2e::environment {

EarthInfrared::EarthInfrared(LocalCelestialInformation* local_celestial_information, SolarRadiationPressureEnvironment* srp_environment)
    : local_celestial_information_(local_celestial_information), srp_environment_(srp_environment) {}

void EarthInfrared::UpdateAllStates() {
  if (!GetIsCalcEarthInfraredEnabled()) return;
  CalcEarthInfrared(local_celestial_information_);
}

std::string EarthInfrared::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += logger::WriteScalar("earth_infrared_temperature_hot_side");
  str_tmp += logger::WriteScalar("earth_infrared_temperature_cold_side");
  str_tmp += logger::WriteScalar("earth_infrared_W_m2");

  return str_tmp;
}

std::string EarthInfrared::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += logger::WriteScalar(earth_infrared_temperature_hot_side_);
  str_tmp += logger::WriteScalar(earth_infrared_temperature_cold_side_);
  str_tmp += logger::WriteScalar(earth_infrared_W_m2_);

  return str_tmp;
}

void EarthInfrared::CalcEarthInfrared(const LocalCelestialInformation* local_celestial_information) {
  math::Vector<3> earth_position_b_m = local_celestial_information->GetPositionFromSpacecraft_b_m("EARTH");
  double earth_distance_m = earth_position_b_m.CalcNorm();
  if (srp_environment_->GetIsEclipsed()) {
    earth_infrared_W_m2_ = stefan_boltzmann_constant_W_m2K4 * std::pow(earth_infrared_temperature_cold_side_, 4) *
                           pow((earth_equatorial_radius_m / earth_distance_m), 2.0) / 4.0;
  } else {
    earth_infrared_W_m2_ = stefan_boltzmann_constant_W_m2K4 * std::pow(earth_infrared_temperature_hot_side_, 4) *
                           pow((earth_equatorial_radius_m / earth_distance_m), 2.0) / 4.0;
  }
}

EarthInfrared InitEarthInfrared(std::string initialize_file_path, LocalCelestialInformation* local_celestial_information,
                                SolarRadiationPressureEnvironment* srp_environment) {
  auto conf = setting_file_reader::IniAccess(initialize_file_path);
  const char* section = "EARTH_INFRARED";

  EarthInfrared earth_infrared(local_celestial_information, srp_environment);
  earth_infrared.SetIsCalcEarthInfraredEnabled(conf.ReadEnable(section, INI_CALC_LABEL));
  earth_infrared.SetEarthTempHotSide(conf.ReadDouble(section, "earth_infrared_temperature_hot_side"));
  earth_infrared.SetEarthTempColdSide(conf.ReadDouble(section, "earth_infrared_temperature_cold_side"));

  return earth_infrared;
}

}  // namespace s2e::environment
