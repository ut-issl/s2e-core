/**
 * @file solar_radiation_pressure_disturbance.cpp
 * @brief Class to calculate the solar radiation pressure disturbance force and torque
 */

#include "solar_radiation_pressure_disturbance.hpp"

#include <cmath>
#include <setting_file_reader/initialize_file_access.hpp>

#include "../logger/log_utility.hpp"

namespace s2e::disturbances {

SolarRadiationPressureDisturbance::SolarRadiationPressureDisturbance(const std::vector<simulation::Surface>& surfaces,
                                                                     const math::Vector<3>& center_of_gravity_b_m, const bool is_calculation_enabled)
    : SurfaceForce(surfaces, center_of_gravity_b_m, is_calculation_enabled) {}

void SolarRadiationPressureDisturbance::Update(const environment::LocalEnvironment& local_environment, const dynamics::Dynamics& dynamics) {
  UNUSED(dynamics);

  math::Vector<3> sun_position_from_sc_b_m = local_environment.GetCelestialInformation().GetPositionFromSpacecraft_b_m("SUN");
  CalcTorqueForce(sun_position_from_sc_b_m, local_environment.GetSolarRadiationPressure().GetPressure_N_m2());
}

void SolarRadiationPressureDisturbance::CalcCoefficients(const math::Vector<3>& input_direction_b, const double item) {
  UNUSED(input_direction_b);

  for (size_t i = 0; i < surfaces_.size(); i++) {  // Calculate for each surface
    double area = surfaces_[i].GetArea_m2();
    double reflectivity = surfaces_[i].GetReflectivity();
    double specularity = surfaces_[i].GetSpecularity();
    normal_coefficients_[i] =
        area * item * ((1.0 + reflectivity * specularity) * pow(cos_theta_[i], 2.0) + 2.0 / 3.0 * reflectivity * (1.0 - specularity) * cos_theta_[i]);
    tangential_coefficients_[i] = area * item * (1.0 - reflectivity * specularity) * cos_theta_[i] * sin_theta_[i];
  }
}

std::string SolarRadiationPressureDisturbance::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += logger::WriteVector("srp_torque", "b", "Nm", 3);
  str_tmp += logger::WriteVector("srp_force", "b", "N", 3);

  return str_tmp;
}

std::string SolarRadiationPressureDisturbance::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += logger::WriteVector(torque_b_Nm_);
  str_tmp += logger::WriteVector(force_b_N_);

  return str_tmp;
}

SolarRadiationPressureDisturbance InitSolarRadiationPressureDisturbance(const std::string initialize_file_path,
                                                                        const std::vector<simulation::Surface>& surfaces,
                                                                        const math::Vector<3>& center_of_gravity_b_m) {
  auto conf = setting_file_reader::IniAccess(initialize_file_path);
  const char* section = "SOLAR_RADIATION_PRESSURE_DISTURBANCE";

  const bool is_calc_enable = conf.ReadEnable(section, INI_CALC_LABEL);
  const bool is_log_enable = conf.ReadEnable(section, INI_LOG_LABEL);

  SolarRadiationPressureDisturbance srp_disturbance(surfaces, center_of_gravity_b_m, is_calc_enable);
  srp_disturbance.is_log_enabled_ = is_log_enable;

  return srp_disturbance;
}

}  // namespace s2e::disturbances
