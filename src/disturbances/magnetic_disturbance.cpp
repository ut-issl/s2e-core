/**
 * @file magnetic_disturbances.cpp
 * @brief Class to calculate the magnetic disturbance torque
 */

#include "magnetic_disturbance.hpp"

#include <setting_file_reader/initialize_file_access.hpp>
#include <utilities/macros.hpp>

#include "../logger/log_utility.hpp"
#include "../math_physics/randomization/global_randomization.hpp"
#include "../math_physics/randomization/normal_randomization.hpp"
#include "../math_physics/randomization/random_walk.hpp"

MagneticDisturbance::MagneticDisturbance(const ResidualMagneticMoment& rmm_params, const bool is_calculation_enabled)
    : Disturbance(is_calculation_enabled, true), residual_magnetic_moment_(rmm_params) {
  rmm_b_Am2_ = residual_magnetic_moment_.GetConstantValue_b_Am2();
}

Vector<3> MagneticDisturbance::CalcTorque_b_Nm(const Vector<3>& magnetic_field_b_nT) {
  CalcRMM();
  torque_b_Nm_ = kMagUnit_ * OuterProduct(rmm_b_Am2_, magnetic_field_b_nT);
  return torque_b_Nm_;
}

void MagneticDisturbance::Update(const LocalEnvironment& local_environment, const Dynamics& dynamics) {
  UNUSED(dynamics);

  CalcTorque_b_Nm(local_environment.GetGeomagneticField().GetGeomagneticField_b_nT());
}

void MagneticDisturbance::CalcRMM() {
  static s2e::math::Vector<3> random_walk_std_dev(residual_magnetic_moment_.GetRandomWalkStandardDeviation_Am2());
  static s2e::math::Vector<3> random_walk_limit(residual_magnetic_moment_.GetRandomWalkLimit_Am2());
  static RandomWalk<3> random_walk(0.1, random_walk_std_dev, random_walk_limit);  // [FIXME] step width is constant
  static randomization::NormalRand normal_random(0.0, residual_magnetic_moment_.GetRandomNoiseStandardDeviation_Am2(),
                                                 global_randomization.MakeSeed());

  rmm_b_Am2_ = residual_magnetic_moment_.GetConstantValue_b_Am2();
  for (int i = 0; i < 3; ++i) {
    rmm_b_Am2_[i] += random_walk[i] + normal_random;
  }
  ++random_walk;  // Update random walk
}

std::string MagneticDisturbance::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("spacecraft_magnetic_moment", "b", "Am2", 3);
  str_tmp += WriteVector("magnetic_disturbance_torque", "b", "Nm", 3);

  return str_tmp;
}

std::string MagneticDisturbance::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(rmm_b_Am2_);
  str_tmp += WriteVector(torque_b_Nm_);

  return str_tmp;
}

MagneticDisturbance InitMagneticDisturbance(const std::string initialize_file_path, const ResidualMagneticMoment& rmm_params) {
  auto conf = IniAccess(initialize_file_path);
  const char* section = "MAGNETIC_DISTURBANCE";

  const bool is_calc_enable = conf.ReadEnable(section, INI_CALC_LABEL);
  MagneticDisturbance mag_disturbance(rmm_params, is_calc_enable);
  mag_disturbance.is_log_enabled_ = conf.ReadEnable(section, INI_LOG_LABEL);

  return mag_disturbance;
}
