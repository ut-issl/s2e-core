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

SolarRadiationPressureEnvironment::SolarRadiationPressureEnvironment(LocalCelestialInformation* local_celestial_information)
    : local_celestial_information_(local_celestial_information) {
  solar_radiation_pressure_N_m2_ = solar_constant_W_m2_ / environment::speed_of_light_m_s;
  shadow_source_name_ = local_celestial_information_->GetGlobalInformation().GetCenterBodyName();
  sun_radius_m_ = local_celestial_information_->GetGlobalInformation().GetMeanRadiusFromName_m("SUN");
}

void SolarRadiationPressureEnvironment::UpdateAllStates() {
  if (!IsCalcEnabled) return;

  UpdatePressure();
  CalcShadowCoefficient(shadow_source_name_);
}

void SolarRadiationPressureEnvironment::UpdatePressure() {
  const libra::Vector<3> r_sc2sun_eci = local_celestial_information_->GetPositionFromSpacecraft_i_m("SUN");
  const double distance_sat_to_sun = CalcNorm(r_sc2sun_eci);
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
    shadow_coefficient_ = 1.0;
    return;
  }

  const libra::Vector<3> r_sc2sun_eci = local_celestial_information_->GetPositionFromSpacecraft_i_m("SUN");
  const libra::Vector<3> r_sc2source_eci = local_celestial_information_->GetPositionFromSpacecraft_i_m(shadow_source_name.c_str());

  const double shadow_source_radius_m = local_celestial_information_->GetGlobalInformation().GetMeanRadiusFromName_m(shadow_source_name.c_str());

  const double distance_sat_to_sun = CalcNorm(r_sc2sun_eci);
  const double sd_sun = asin(sun_radius_m_ / distance_sat_to_sun);                    // Apparent radius of the sun
  const double sd_source = asin(shadow_source_radius_m / CalcNorm(r_sc2source_eci));  // Apparent radius of the shadow source

  // Angle of deviation from shadow source center to sun center
  const double delta =
      acos(InnerProduct(r_sc2source_eci, r_sc2sun_eci - r_sc2source_eci) / CalcNorm(r_sc2source_eci) / CalcNorm(r_sc2sun_eci - r_sc2source_eci));
  // The angle between the center of the sun and the common chord
  const double x = (delta * delta + sd_sun * sd_sun - sd_source * sd_source) / (2.0 * delta);
  // The length of the common chord of the apparent solar disk and apparent telestial disk
  const double y = sqrt(std::max(sd_sun * sd_sun - x * x, 0.0));

  const double a = sd_sun;
  const double b = sd_source;
  const double c = delta;

  if (c < fabs(a - b) && a <= b)  // The occultation is total (spacecraft is in umbra)
  {
    shadow_coefficient_ = 0.0;
  } else if (c < fabs(a - b) && a > b)  // The occultation is partial but maximum
  {
    shadow_coefficient_ = 1.0 - (b * b) / (a * a);
  } else if (fabs(a - b) <= c && c <= (a + b))  // spacecraft is in penumbra
  {
    double A = a * a * acos(x / a) + b * b * acos((c - x) / b) - c * y;  // The area of the occulted segment of the apparent solar disk
    shadow_coefficient_ = 1.0 - A / (libra::pi * a * a);
  } else {  // no occultation takes place
    assert(c > (a + b));
    shadow_coefficient_ = 1.0;
  }
}
