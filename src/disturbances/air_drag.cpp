/**
 * @file air_drag.cpp
 * @brief Class to calculate the air drag disturbance force and torque
 */

#include "air_drag.hpp"

#include <cmath>
#include <environment/global/physical_constants.hpp>
#include <iostream>
#include <library/math/constants.hpp>

#include "../interface/log_output/log_utility.hpp"

AirDrag::AirDrag(const vector<Surface>& surfaces, const libra::Vector<3>& center_of_gravity_b_m, const double wall_temperature_degC,
                 const double molecular_temperature_degC, const double molecular_weight, const bool is_calculation_enabled)
    : SurfaceForce(surfaces, center_of_gravity_b_m, is_calculation_enabled) {
  int num = surfaces_.size();
  Ct_.assign(num, 1.0);
  Cn_.assign(num, 0.0);
  Tw_ = wall_temperature_degC;
  Tm_ = molecular_temperature_degC;
  M_ = molecular_weight;
}

void AirDrag::Update(const LocalEnvironment& local_environment, const Dynamics& dynamics) {
  double air_dens = local_environment.GetAtmosphere().GetAirDensity();
  Vector<3> tmp = dynamics.GetOrbit().GetSatVelocity_b();
  CalcTorqueForce(tmp, air_dens);
}

void AirDrag::CalcCoefficients(libra::Vector<3>& velocity_b_m_s, double air_dens) {
  double velocity_norm_m_s = norm(velocity_b_m_s);
  rho_kg_m3_ = air_dens;
  CalCnCt(velocity_b_m_s);
  for (size_t i = 0; i < surfaces_.size(); i++) {
    double k = 0.5 * rho_kg_m3_ * velocity_norm_m_s * velocity_norm_m_s * surfaces_[i].GetArea();
    normal_coefficients_[i] = k * Cn_[i];
    tangential_coefficients_[i] = k * Ct_[i];
  }
}

double AirDrag::CalcFuncPi(double s) {
  double x;
  double erfs = erf(s);  // ERF function is defined in math standard library
  x = s * exp(-s * s) + sqrt(libra::pi) * (s * s + 0.5) * (1.0 + erfs);
  return x;
}

double AirDrag::CalcFuncChi(double s) {
  double x;
  double erfs = erf(s);
  x = exp(-s * s) + sqrt(libra::pi) * s * (1.0 + erfs);
  return x;
}

void AirDrag::CalCnCt(Vector<3>& velocity_b_m_s) {
  double velocity_norm_m_s = norm(velocity_b_m_s);

  // Re-emitting speed
  double S = sqrt(M_ * velocity_norm_m_s * velocity_norm_m_s / (2.0 * environment::boltzmann_constant_J_K * Tw_));
  // CalcTheta(vel_b);
  for (size_t i = 0; i < surfaces_.size(); i++) {
    double Sn = S * cos_theta_[i];
    double St = S * sin_theta_[i];
    double diffuse = 1.0 - surfaces_[i].GetAirSpecularity();
    Cn_[i] = (2.0 - diffuse) / sqrt(libra::pi) * CalcFuncPi(Sn) / (S * S) + diffuse / 2.0 * CalcFuncChi(Sn) / (S * S) * sqrt(Tw_ / Tm_);
    Ct_[i] = diffuse * St * CalcFuncChi(Sn) / (sqrt(libra::pi) * S * S);
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
