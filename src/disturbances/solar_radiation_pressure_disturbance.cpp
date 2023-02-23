/**
 * @file solar_radiation_pressure_disturbance.cpp
 * @brief Class to calculate the solar radiation pressure disturbance force and torque
 */

#include "solar_radiation_pressure_disturbance.hpp"

#include <cmath>

#include "../library/logger/log_utility.hpp"

SolarRadiationPressureDisturbance::SolarRadiationPressureDisturbance(const vector<Surface>& surfaces, const libra::Vector<3>& center_of_gravity_b_m,
                                                                     const bool is_calculation_enabled)
    : SurfaceForce(surfaces, center_of_gravity_b_m, is_calculation_enabled) {}

void SolarRadiationPressureDisturbance::Update(const LocalEnvironment& local_env, const Dynamics& dynamics) {
  UNUSED(dynamics);

  libra::Vector<3> sun_position_from_sc_b_m = local_env.GetCelestialInformation().GetPositionFromSpacecraft_b_m("SUN");
  CalcTorqueForce(sun_position_from_sc_b_m, local_env.GetSolarRadiationPressure().GetPressure_N_m2());
}

void SolarRadiationPressureDisturbance::CalcCoefficients(const libra::Vector<3>& input_direction_b, const double item) {
  UNUSED(input_direction_b);

  for (size_t i = 0; i < surfaces_.size(); i++) {  // Calculate for each surface
    double area = surfaces_[i].GetArea();
    double reflectivity = surfaces_[i].GetReflectivity();
    double specularity = surfaces_[i].GetSpecularity();
    normal_coefficients_[i] =
        area * item * ((1.0 + reflectivity * specularity) * pow(cos_theta_[i], 2.0) + 2.0 / 3.0 * reflectivity * (1.0 - specularity) * cos_theta_[i]);
    tangential_coefficients_[i] = area * item * (1.0 - reflectivity * specularity) * cos_theta_[i] * sin_theta_[i];
  }
}

std::string SolarRadiationPressureDisturbance::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("srp_torque", "b", "Nm", 3);
  str_tmp += WriteVector("srp_force", "b", "N", 3);

  return str_tmp;
}

std::string SolarRadiationPressureDisturbance::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(torque_b_Nm_);
  str_tmp += WriteVector(force_b_N_);

  return str_tmp;
}
