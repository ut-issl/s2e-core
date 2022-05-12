#include "MagDisturbance.h"

#include "../Library/math/NormalRand.hpp"
using libra::NormalRand;
#include <Library/utils/Macros.hpp>

#include "../Interface/LogOutput/LogUtility.h"
#include "../Library/math/GlobalRand.h"
#include "../Library/math/RandomWalk.hpp"

using namespace std;

MagDisturbance::MagDisturbance(const Vector<3>& rmm_const_b, const double rmm_rwdev, const double rmm_rwlimit, const double rmm_wnvar)
    : rmm_const_b_(rmm_const_b), rmm_rwdev_(rmm_rwdev), rmm_rwlimit_(rmm_rwlimit), rmm_wnvar_(rmm_wnvar) {
  for (int i = 0; i < 3; ++i) {
    torque_b_[i] = 0;
  }                    //残留磁気トルク値の初期化
  mag_unit_ = 1.0E-9;  //磁気が[nT]単位であることによる単位合わせ([nT]→[T])
  rmm_b_ = rmm_const_b_;
}

//残留磁気トルク（磁気外乱）の計算
Vector<3> MagDisturbance::CalcTorque(const Vector<3>& mag_b) {
  CalcRMM();
  torque_b_ = mag_unit_ * outer_product(rmm_b_, mag_b);
  return torque_b_;
}

void MagDisturbance::Update(const LocalEnvironment& local_env, const Dynamics& dynamics) {
  UNUSED(dynamics);

  CalcTorque(local_env.GetMag().GetMag_b());
}

//残留磁気モーメントの計算
void MagDisturbance::CalcRMM() {
  static Vector<3> stddev(rmm_rwdev_);
  static Vector<3> limit(rmm_rwlimit_);
  static RandomWalk<3> rw(0.1, stddev, limit);
  static NormalRand nr(0.0, rmm_wnvar_, g_rand.MakeSeed());

  rmm_b_ = rmm_const_b_;
  for (int i = 0; i < 3; ++i) {
    rmm_b_[i] += rw[i] + nr;
  }
  ++rw;  //ランダムウォーク更新
}

//コンソール表示用関数
void MagDisturbance::PrintTorque() {
  cout << "MgDist_Torque_b =(" << torque_b_[0] << "," << torque_b_[1] << "," << torque_b_[2] << ") Nm";
  cout << endl;
}

string MagDisturbance::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("rmm", "b", "Am^2", 3);
  str_tmp += WriteVector("mag_dist_torque", "b", "Nm", 3);

  return str_tmp;
}

string MagDisturbance::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(rmm_b_);
  str_tmp += WriteVector(torque_b_);

  return str_tmp;
}
