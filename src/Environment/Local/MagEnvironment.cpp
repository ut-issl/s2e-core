#include "MagEnvironment.h"
#include "../../Interface/InitInput/IniAccess.h"
#include "../../Library/igrf/igrf.h"
#include "../../Library/math/GlobalRand.h"
#include "../../Library/math/NormalRand.hpp"
#include "../../Library/math/RandomWalk.hpp"

using libra::NormalRand;
using namespace std;

MagEnvironment::MagEnvironment(string fname, double mag_rwdev,
                               double mag_rwlimit, double mag_wnvar)
    : fname_(fname), mag_rwdev_(mag_rwdev), mag_rwlimit_(mag_rwlimit),
      mag_wnvar_(mag_wnvar) {
  //初期化コンストラクタ
  for (int i = 0; i < 3; ++i) {
    Mag_i_[i] = 0;
  }
  for (int i = 0; i < 3; ++i) {
    Mag_b_[i] = 0;
  }
  set_file_path(fname_.c_str());
}

void MagEnvironment::CalcMag(double decyear, double side, Vector<3> lat_lon_alt,
                             Quaternion q_i2b) {
  if (!IsCalcEnabled)
    return;

  double latrad = lat_lon_alt(0);
  double lonrad = lat_lon_alt(1);
  double alt = lat_lon_alt(2);

  double mag_i_array[3];
  IgrfCalc(decyear, latrad, lonrad, alt, side, mag_i_array);
  AddNoise(mag_i_array);
  for (int i = 0; i < 3; ++i) {
    Mag_i_[i] = mag_i_array[i];
  }
  Mag_b_ = q_i2b.frame_conv(Mag_i_);
}

//磁場真値のIGRFからのズレを加える関数
void MagEnvironment::AddNoise(double *mag_i_array) {
  static Vector<3> stddev(mag_rwdev_);
  static Vector<3> limit(mag_rwlimit_);
  static RandomWalk<3> rw(0.1, stddev, limit);
  static NormalRand nr(0.0, mag_wnvar_, g_rand.MakeSeed());
  for (int i = 0; i < 3; ++i) {
    mag_i_array[i] += rw[i] + nr;
  }
  ++rw; // ランダムウォーク更新
}

//地磁気ベクトル(ECI座標系)[nT]の取得用関数
Vector<3> MagEnvironment::GetMag_i() const { return Mag_i_; }

//地磁気ベクトル(body座標系)[nT]の取得用関数
Vector<3> MagEnvironment::GetMag_b() const { return Mag_b_; }

string MagEnvironment::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("mag", "i", "nT", 3);
  str_tmp += WriteVector("mag", "b", "nT", 3);

  return str_tmp;
}

string MagEnvironment::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(Mag_i_);
  str_tmp += WriteVector(Mag_b_);

  return str_tmp;
}