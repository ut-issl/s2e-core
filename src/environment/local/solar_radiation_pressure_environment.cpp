/**
 * @file solar_radiation_pressure_environment.cpp
 * @brief Class to calculate Solar Radiation Pressure
 */
#include "solar_radiation_pressure_environment.hpp"

#include <algorithm>
#include <cassert>
#include <fstream>

#include "library/logger/log_utility.hpp"
#include "library/math/constants.hpp"
#include "library/math/vector.hpp"
#include "setting_file_reader/initialize_file_access.hpp"

SolarRadiationPressureEnvironment::SolarRadiationPressureEnvironment(LocalCelestialInformation* local_celestial_information)
    : local_celestial_information_(local_celestial_information) {
  solar_radiation_pressure_N_m2_ = solar_constant_W_m2_ / environment::speed_of_light_m_s;
  shadow_source_name_list_.push_back(local_celestial_information_->GetGlobalInformation().GetCenterBodyName());
  sun_radius_m_ = local_celestial_information_->GetGlobalInformation().GetMeanRadiusFromName_m("SUN");
}

void SolarRadiationPressureEnvironment::UpdateAllStates() {
  if (!IsCalcEnabled) return;

  UpdatePressure();
  shadow_coefficient_ = 1.0;  // Initialize for multiple shadow source
  for (auto shadow_source_name : shadow_source_name_list_) {
    CalcShadowCoefficient(shadow_source_name);
  }
}

void SolarRadiationPressureEnvironment::UpdatePressure() {
  const libra::Vector<3> r_sc2sun_eci = local_celestial_information_->GetPositionFromSpacecraft_i_m("SUN");
  const double distance_sat_to_sun = r_sc2sun_eci.CalcNorm();
  solar_radiation_pressure_N_m2_ =
      solar_constant_W_m2_ / environment::speed_of_light_m_s / pow(distance_sat_to_sun / environment::astronomical_unit_m, 2.0);
}

std::string SolarRadiationPressureEnvironment::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar("solar_radiation_pressure_at_spacecraft_position", "N/m2");
  str_tmp += WriteScalar("shadow_coefficient_at_spacecraft_position");

  return str_tmp;
}

std::string SolarRadiationPressureEnvironment::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(solar_radiation_pressure_N_m2_ * shadow_coefficient_);
  str_tmp += WriteScalar(shadow_coefficient_);

  return str_tmp;
}

void SolarRadiationPressureEnvironment::CalcShadowCoefficient(std::string shadow_source_name) {
  if (shadow_source_name == "SUN") {
    shadow_coefficient_ *= 1.0;
    return;
  }

  const libra::Vector<3> r_sc2sun_eci = local_celestial_information_->GetPositionFromSpacecraft_i_m("SUN");
  const libra::Vector<3> r_sc2source_eci = local_celestial_information_->GetPositionFromSpacecraft_i_m(shadow_source_name.c_str());

  const double shadow_source_radius_m = local_celestial_information_->GetGlobalInformation().GetMeanRadiusFromName_m(shadow_source_name.c_str());

  const double distance_sat_to_sun = r_sc2sun_eci.CalcNorm();
  const double sd_sun = asin(sun_radius_m_ / distance_sat_to_sun);                     // Apparent radius of the sun
  const double sd_source = asin(shadow_source_radius_m / r_sc2source_eci.CalcNorm());  // Apparent radius of the shadow source

  // Angle of deviation from shadow source center to sun center
  libra::Vector<3> r_source2sun_eci = r_sc2sun_eci - r_sc2source_eci;
  const double delta = acos(InnerProduct(r_sc2source_eci, r_sc2sun_eci - r_sc2source_eci) / r_sc2source_eci.CalcNorm() / r_source2sun_eci.CalcNorm());
  // The angle between the center of the sun and the common chord
  const double x = (delta * delta + sd_sun * sd_sun - sd_source * sd_source) / (2.0 * delta);
  // The length of the common chord of the apparent solar disk and apparent telestial disk
  const double y = sqrt(std::max(sd_sun * sd_sun - x * x, 0.0));

  const double a = sd_sun;
  const double b = sd_source;
  const double c = delta;

  if (c < fabs(a - b) && a <= b)  // The occultation is total (spacecraft is in umbra)
  {
    shadow_coefficient_ *= 0.0;
  } else if (c < fabs(a - b) && a > b)  // The occultation is partial but maximum
  {
    shadow_coefficient_ = 1.0 - (b * b) / (a * a);
  } else if (fabs(a - b) <= c && c <= (a + b))  // spacecraft is in penumbra
  {
    double A = a * a * acos(x / a) + b * b * acos((c - x) / b) - c * y;  // The area of the occulted segment of the apparent solar disk
    shadow_coefficient_ *= 1.0 - A / (libra::pi * a * a);
  } else {  // no occultation takes place
    if (c < (a + b)) {
      std::cout << "[Error SRP Environment]: The calculation error was occurred at the shadow calculation." << std::endl;
      std::cout << "                         The orbit setting may have something wrong." << std::endl;
      std::exit(1);
    }
    shadow_coefficient_ *= 1.0;
  }
}

SolarRadiationPressureEnvironment InitSolarRadiationPressureEnvironment(std::string initialize_file_path,
                                                                        LocalCelestialInformation* local_celestial_information) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "SOLAR_RADIATION_PRESSURE_ENVIRONMENT";

  SolarRadiationPressureEnvironment srp_env(local_celestial_information);
  srp_env.IsCalcEnabled = conf.ReadEnable(section, INI_CALC_LABEL);
  srp_env.is_log_enabled_ = conf.ReadEnable(section, INI_LOG_LABEL);

  size_t number_of_third_shadow_source = conf.ReadInt(section, "number_of_third_shadow_source");
  std::vector<std::string> list = conf.ReadVectorString(section, "third_shadow_source_name", number_of_third_shadow_source);
  for (size_t i = 0; i < number_of_third_shadow_source; i++) {
    srp_env.AddShadowSource(list[0]);
  }

  return srp_env;
}
