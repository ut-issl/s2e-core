/*!
  \file   MagSensor.cpp
  \author KUDO Takumi
  \date   Sat May 14 14:14:23 2011
  \brief  MagSen.cppの実装
*/
#include "MagSensor.h"

using namespace libra;
using namespace std;
#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/GlobalRand.h"
#include "../../Interface/LogOutput/Logger.h"


MagSensor::MagSensor(const int sensor_id,
  const Quaternion& q_b2c,
  const Matrix<3, 3>& scale_factor,
  const Vector<3>& bias_c,
  double rw_stepwidth,
  const Vector<3>& rw_stddev_c,
  const Vector<3>& rw_limit_c,
  const Vector<3>& nr_stddev_c,
  const MagEnvironment *magnet)
  : ComponentBase(60), sensor_id_(sensor_id), q_b2c_(q_b2c),
  scale_factor_(scale_factor), bias_c_(bias_c),
  n_rw_c_(rw_stepwidth, rw_stddev_c, rw_limit_c),
  nrs0_c_(0.0, nr_stddev_c[0], g_rand.MakeSeed()),
  nrs1_c_(0.0, nr_stddev_c[1], g_rand.MakeSeed()),
  nrs2_c_(0.0, nr_stddev_c[2], g_rand.MakeSeed()),
  magnet_(magnet)
{
}
void MagSensor::MainRoutine(int count)
{
	measure(magnet_->GetMag_b());
}

Vector<3> MagSensor::measure(const Vector<3>& mag_b)
{
  // 磁気ベクトルを機体座標系からセンサ実座標系へ変換
  mag_c = q_b2c_.frame_conv(mag_b);

  // スケールファクタの加味
  mag_c = scale_factor_  * mag_c;
  // バイアス定常成分加算
  mag_c += bias_c_;
  // ランダムウォークの加算
  for (int i = 0; i < 3; ++i) { mag_c[i] += n_rw_c_[i]; }
  // ランダムウォークの更新
  ++n_rw_c_;
  // Gaussianノイズの加算
  mag_c[0] += nrs0_c_;
  mag_c[1] += nrs1_c_;
  mag_c[2] += nrs2_c_;

  return mag_c;
}

void MagSensor::PrintParams(int sensor_id)
{
  cout << "magsensor" << sensor_id << "\n";
  cout << "q_b2c =(" << q_b2c_[0] << "," << q_b2c_[1] << "," << q_b2c_[2] << "," << q_b2c_[3] << ") \n";
  cout << "ScaleFactor[0] =(" << scale_factor_[0][0] << "," << scale_factor_[0][1] << "," << scale_factor_[0][2] << ")  \n";
  cout << "ScaleFactor[1] =(" << scale_factor_[1][0] << "," << scale_factor_[1][1] << "," << scale_factor_[1][2] << ")  \n";
  cout << "ScaleFactor[2] =(" << scale_factor_[2][0] << "," << scale_factor_[2][1] << "," << scale_factor_[2][2] << ")  \n";
  cout << "bias_c =(" << bias_c_[0] << "," << bias_c_[1] << "," << bias_c_[2] << ")  [nT]\n";
}


string MagSensor::GetLogHeader() const
{
  string str_tmp = "";
  const string st_sensor_id = std::to_string(static_cast<long long>(sensor_id_));
  const char *cs = st_sensor_id.data();
  string MSSection = "mag_sensor";
  str_tmp += WriteVector(MSSection+cs, "c", "nT", 3);

  return str_tmp;
}

string MagSensor::GetLogValue() const
{
  string str_tmp = "";

  str_tmp += WriteVector(mag_c);


  return str_tmp;
}