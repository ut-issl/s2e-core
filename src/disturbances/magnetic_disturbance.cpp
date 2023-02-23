/**
 * @file magnetic_disturbances.cpp
 * @brief Class to calculate the magnetic disturbance torque
 */

#include "magnetic_disturbance.hpp"

#include <library/utilities/macros.hpp>

#include "../library/logger/log_utility.hpp"
#include "../library/randomization/global_randomization.hpp"
#include "../library/randomization/normal_randomization.hpp"
#include "../library/randomization/random_walk.hpp"

MagDisturbance::MagDisturbance(const RMMParams& rmm_params, const bool is_calculation_enabled)
    : SimpleDisturbance(is_calculation_enabled), rmm_params_(rmm_params) {
  rmm_b_Am2_ = rmm_params_.GetRMMConst_b();
}

Vector<3> MagDisturbance::CalcTorque_b_Nm(const Vector<3>& magnetic_field_b_nT) {
  CalcRMM();
  torque_b_Nm_ = kMagUnit_ * outer_product(rmm_b_Am2_, magnetic_field_b_nT);
  return torque_b_Nm_;
}

void MagDisturbance::Update(const LocalEnvironment& local_environment, const Dynamics& dynamics) {
  UNUSED(dynamics);

  CalcTorque_b_Nm(local_environment.GetMag().GetMagneticField_b_nT());
}

void MagDisturbance::CalcRMM() {
  static libra::Vector<3> random_walk_std_dev(rmm_params_.GetRMMRWDev());
  static libra::Vector<3> random_walk_limit(rmm_params_.GetRMMRWLimit());
  static RandomWalk<3> random_walk(0.1, random_walk_std_dev, random_walk_limit);  // [FIXME] step width is constant
  static libra::NormalRand normal_random(0.0, rmm_params_.GetRMMWNVar(), g_rand.MakeSeed());

  rmm_b_Am2_ = rmm_params_.GetRMMConst_b();
  for (int i = 0; i < 3; ++i) {
    rmm_b_Am2_[i] += random_walk[i] + normal_random;
  }
  ++random_walk;  // Update random walk
}

std::string MagDisturbance::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteVector("spacecraft_magnetic_moment", "b", "Am2", 3);
  str_tmp += WriteVector("magnetic_disturbance_torque", "b", "Nm", 3);

  return str_tmp;
}

std::string MagDisturbance::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(rmm_b_Am2_);
  str_tmp += WriteVector(torque_b_Nm_);

  return str_tmp;
}
