/*!
\file   MagSensor.cpp
\author KUDO Takumi
\date   Sat May 14 14:14:23 2011
\brief  Gyro.cppの実装
*/
#include "Gyro.h"
#include "../CDH/OBC_C2A.h"
#include "../../Library/utils/endian.h"


using namespace libra;
using namespace std;
#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/GlobalRand.h"

Gyro::Gyro(ClockGenerator* clock_gen,
  const int sensor_id,
  const int port_id,
  const Quaternion& q_b2c,
  const Matrix<3, 3>& scale_factor,
  const Vector<3>& bias_c,
  double rw_stepwidth,
  const Vector<3>& rw_stddev_c,
  const Vector<3>& rw_limit_c,
  const Vector<3>& nr_stddev_c,
  double range_to_const_c,
  double range_to_zero_c,
  double current,
  const Dynamics *dynamics)
  : ComponentBase(50, clock_gen), sensor_id_(sensor_id), port_id_(port_id), q_b2c_(q_b2c),
  scale_factor_(scale_factor), bias_c_(bias_c), n_rw_c_(rw_stepwidth, rw_stddev_c, rw_limit_c),
  nrs0_c_(0.0, nr_stddev_c[0], g_rand.MakeSeed()), 
  nrs1_c_(0.0, nr_stddev_c[1], g_rand.MakeSeed()),
  nrs2_c_(0.0, nr_stddev_c[2], g_rand.MakeSeed()),
  range_to_const_c_(range_to_const_c), range_to_zero_c_(range_to_zero_c),
  current_(current),dynamics_(dynamics)
{
}

Gyro::~Gyro()
{
}

void Gyro::MainRoutine(int count)
{
  measure(dynamics_->GetAttitude().GetOmega_b());
}

double Gyro::GetCurrent(int port_id) const
{
  if (!isOn_) return 0;
  return current_;
}

//body座標系角速度を入力，
Vector<3> Gyro::measure(const Vector<3>& omega_b)
{
  //レンジチェック
  Gyro::RangeCheck();

  // 角速度をを機体座標系からセンサ実座標系へ変換
  omega_c = q_b2c_.frame_conv(omega_b);

  // スケールファクタの加味
  omega_c = scale_factor_  * omega_c;
  // バイアス定常成分加算
  omega_c += bias_c_;
  // ランダムウォークの加算
  for (int i = 0; i < 3; ++i) { omega_c[i] += n_rw_c_[i]; }
  // ランダムウォークの更新
  ++n_rw_c_;
  // Gaussianノイズの加算
  omega_c[0] += nrs0_c_;
  omega_c[1] += nrs1_c_;
  omega_c[2] += nrs2_c_;

  //コンポ角速度の値がrangeに引っかかっていた場合，ヘッダファイルのrangeに記載した措置を行う．
  Gyro::Clip();
  return omega_c;
}

void Gyro::RangeCheck()
{
  if (range_to_const_c_ < 0.0 || range_to_zero_c_ < 0.0)  //両方正の数でなければならない
  {
    throw invalid_argument("range should be positive!!");
  }

  if (range_to_const_c_ > range_to_zero_c_)  //range_to_const_c_ <= range_to_zero_c_　でなければならない．
  {
    throw invalid_argument("range2 should be greater than range1!!");
  }
}

void Gyro::Clip()
{
  for (size_t i = 0; i < omega_c.dim(); ++i)
  {
    if (omega_c[i] >= range_to_const_c_  && omega_c[i] < range_to_zero_c_)
    {
      omega_c[i] = range_to_const_c_;
    }
    else if (omega_c[i] <= -range_to_const_c_ && omega_c[i] > -range_to_zero_c_)
    {
      omega_c[i] = -range_to_const_c_;
    }
    else if (fabs(omega_c[i]) >= range_to_zero_c_)
    {
      omega_c[i] = 0.0;
    }
  }
}


void Gyro::PrintParams(int sensor_id)
{
  cout << "magsensor" << sensor_id << "\n";
  cout << "q_b2c =(" << q_b2c_[0] << "," << q_b2c_[1] << "," << q_b2c_[2] << "," << q_b2c_[3] << ") \n";
  cout << "ScaleFactor[0] =(" << scale_factor_[0][0] << "," << scale_factor_[0][1] << "," << scale_factor_[0][2] << ")  \n";
  cout << "ScaleFactor[1] =(" << scale_factor_[1][0] << "," << scale_factor_[1][1] << "," << scale_factor_[1][2] << ")  \n";
  cout << "ScaleFactor[2] =(" << scale_factor_[2][0] << "," << scale_factor_[2][1] << "," << scale_factor_[2][2] << ")  \n";
  cout << "bias_c =(" << bias_c_[0] << "," << bias_c_[1] << "," << bias_c_[2] << ")  [nT]\n";
}

double Gyro::GetRangeToConst() {
	return range_to_const_c_;
}

string Gyro::GetLogHeader() const
{
  string str_tmp = "";
  const string st_sensor_id = std::to_string(static_cast<long long>(sensor_id_));
  const char *cs = st_sensor_id.data();
  string GSection = "gyro_omega";
  str_tmp += WriteVector(GSection + cs, "c", "rad/s", 3);

  return str_tmp;
}

string Gyro::GetLogValue() const
{
  string str_tmp = "";

  str_tmp += WriteVector(omega_c);

  return str_tmp;
}
