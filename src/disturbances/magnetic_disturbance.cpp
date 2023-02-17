/**
 * @file magnetic_disturbances.cpp
 * @brief Class to calculate the magnetic disturbance torque
 */

#include "magnetic_disturbance.hpp"

#include "../library/randomization/normal_randomization.hpp"
using libra::NormalRand;
#include <library/utilities/macros.hpp>

#include "../library/logger/log_utility.hpp"
#include "../library/randomization/global_randomization.hpp"
#include "../library/randomization/random_walk.hpp"

using namespace std;

MagDisturbance::MagDisturbance(const RMMParams& rmm_params) : rmm_params_(rmm_params) {
  for (int i = 0; i < 3; ++i) {
    torque_b_[i] = 0;
  }
  mag_unit_ = 1.0E-9;  // [nT] -> [T]
  rmm_b_ = rmm_params_.GetRMMConst_b();
}

Vector<3> MagDisturbance::CalcTorque(const Vector<3>& mag_b) {
  CalcRMM();
  torque_b_ = mag_unit_ * outer_product(rmm_b_, mag_b);
  return torque_b_;
}

void MagDisturbance::Update(const LocalEnvironment& local_env, const Dynamics& dynamics) {
  UNUSED(dynamics);

  CalcTorque(local_env.GetMag().GetMag_b());
}

void MagDisturbance::CalcRMM() {
  static Vector<3> stddev(rmm_params_.GetRMMRWDev());
  static Vector<3> limit(rmm_params_.GetRMMRWLimit());
  static RandomWalk<3> rw(0.1, stddev, limit);
  static NormalRand nr(0.0, rmm_params_.GetRMMWNVar(), g_rand.MakeSeed());

  rmm_b_ = rmm_params_.GetRMMConst_b();
  for (int i = 0; i < 3; ++i) {
    rmm_b_[i] += rw[i] + nr;
  }
  ++rw;  // Update random walk
}

void MagDisturbance::PrintTorque() {
  cout << "MgDist_Torque_b =(" << torque_b_[0] << "," << torque_b_[1] << "," << torque_b_[2] << ") Nm";
  cout << endl;
}

string MagDisturbance::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("spacecraft_magnetic_moment", "b", "Am2", 3);
  str_tmp += WriteVector("magnetic_disturbance_torque", "b", "Nm", 3);

  return str_tmp;
}

string MagDisturbance::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(rmm_b_);
  str_tmp += WriteVector(torque_b_);

  return str_tmp;
}
