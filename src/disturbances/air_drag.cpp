/**
 * @file air_drag.cpp
 * @brief Class to calculate the air drag disturbance force and torque
 */

#include "air_drag.hpp"

#include <cmath>
#include <environment/global/physical_constants.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <library/math/constants.hpp>

#include "../library/logger/log_utility.hpp"

AirDrag::AirDrag(const std::vector<Surface>& surfaces, const libra::Vector<3>& center_of_gravity_b_m, const double wall_temperature_K,
                 const double molecular_temperature_K, const double molecular_weight_g_mol, const bool is_calculation_enabled)
    : SurfaceForce(surfaces, center_of_gravity_b_m, is_calculation_enabled),
      wall_temperature_K_(wall_temperature_K),
      molecular_temperature_K_(molecular_temperature_K),
      molecular_weight_g_mol_(molecular_weight_g_mol) {
  size_t num = surfaces_.size();
  ct_.assign(num, 1.0);
  cn_.assign(num, 0.0);
}

void AirDrag::Update(const LocalEnvironment& local_environment, const Dynamics& dynamics) {
  double air_density_kg_m3 = local_environment.GetAtmosphere().GetAirDensity_kg_m3();

  libra::Matrix<3, 3> dcm_ecef2eci =
      local_environment.GetCelestialInformation().GetGlobalInformation().GetEarthRotation().GetDcmJ2000ToEcef().Transpose();
  libra::Vector<3> relative_velocity_wrt_atmosphere_i_m_s = dcm_ecef2eci * dynamics.GetOrbit().GetVelocity_ecef_m_s();
  libra::Quaternion quaternion_i2b = dynamics.GetAttitude().GetQuaternion_i2b();
  libra::Vector<3> velocity_b_m_s = quaternion_i2b.FrameConversion(relative_velocity_wrt_atmosphere_i_m_s);
  CalcTorqueForce(velocity_b_m_s, air_density_kg_m3);
}

void AirDrag::CalcCoefficients(const libra::Vector<3>& velocity_b_m_s, const double air_density_kg_m3) {
  double velocity_norm_m_s = velocity_b_m_s.CalcNorm();
  CalcCnCt(velocity_b_m_s);
  for (size_t i = 0; i < surfaces_.size(); i++) {
    double k = 0.5 * air_density_kg_m3 * velocity_norm_m_s * velocity_norm_m_s * surfaces_[i].GetArea_m2();
    normal_coefficients_[i] = k * cn_[i];
    tangential_coefficients_[i] = k * ct_[i];
  }
}

double AirDrag::CalcFunctionPi(const double s) {
  double x;
  double erfs = erf(s);  // ERF function is defined in math standard library
  x = s * exp(-s * s) + sqrt(libra::pi) * (s * s + 0.5) * (1.0 + erfs);
  return x;
}

double AirDrag::CalcFunctionChi(const double s) {
  double x;
  double erfs = erf(s);
  x = exp(-s * s) + sqrt(libra::pi) * s * (1.0 + erfs);
  return x;
}

void AirDrag::CalcCnCt(const Vector<3>& velocity_b_m_s) {
  double velocity_norm_m_s = velocity_b_m_s.CalcNorm();

  // Re-emitting speed
  double speed =
      sqrt(molecular_weight_g_mol_ * velocity_norm_m_s * velocity_norm_m_s / (2.0 * environment::boltzmann_constant_J_K * wall_temperature_K_));
  // CalcTheta(vel_b);
  for (size_t i = 0; i < surfaces_.size(); i++) {
    double speed_n = speed * cos_theta_[i];
    double speed_t = speed * sin_theta_[i];
    double diffuse = 1.0 - surfaces_[i].GetAirSpecularity();
    cn_[i] = (2.0 - diffuse) / sqrt(libra::pi) * CalcFunctionPi(speed_n) / (speed * speed) +
             diffuse / 2.0 * CalcFunctionChi(speed_n) / (speed * speed) * sqrt(wall_temperature_K_ / molecular_temperature_K_);
    ct_[i] = diffuse * speed_t * CalcFunctionChi(speed_n) / (sqrt(libra::pi) * speed * speed);
  }
}

std::string AirDrag::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("air_drag_torque", "b", "Nm", 3);
  str_tmp += WriteVector("air_drag_force", "b", "N", 3);

  return str_tmp;
}

std::string AirDrag::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(torque_b_Nm_);
  str_tmp += WriteVector(force_b_N_);

  return str_tmp;
}

AirDrag InitAirDrag(const std::string initialize_file_path, const std::vector<Surface>& surfaces, const Vector<3>& center_of_gravity_b_m) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "AIR_DRAG";

  const double wall_temperature_K = conf.ReadDouble(section, "wall_temperature_degC") + 273.0;
  const double molecular_temperature_K = conf.ReadDouble(section, "molecular_temperature_degC") + 273.0;
  const double molecular_weight_g_mol = conf.ReadDouble(section, "molecular_weight_g_mol");

  const bool is_calc_enable = conf.ReadEnable(section, INI_CALC_LABEL);
  const bool is_log_enable = conf.ReadEnable(section, INI_LOG_LABEL);

  AirDrag air_drag(surfaces, center_of_gravity_b_m, wall_temperature_K, molecular_temperature_K, molecular_weight_g_mol, is_calc_enable);
  air_drag.is_log_enabled_ = is_log_enable;

  return air_drag;
}
