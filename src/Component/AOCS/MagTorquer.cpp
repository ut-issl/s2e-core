#include "MagTorquer.h"
using namespace libra;
using namespace std;
#include "../../Library/math/GlobalRand.h"
#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/MatVec.hpp"
#include "../../Interface/LogOutput/Logger.h"

MagTorquer::MagTorquer(const int sensor_id,
  const Quaternion& q_b2c,
  const libra::Vector<3>& max_c,
  const libra::Vector<3>& min_c,
  const libra::Vector<3>& bias_c,
  double rw_stepwidth,
  const libra::Vector<3>& rw_stddev_c,
  const libra::Vector<3>& rw_limit_c,
  const libra::Vector<3>& nr_stddev_c,
  unsigned int resolution)
  : ComponentBase(40), sensor_id_(sensor_id), q_b2c_(q_b2c), q_c2b_(q_b2c_.conjugate()), resolution_(resolution),
  max_c_(max_c), min_c_(min_c), bias_c_(bias_c), n_rw_c_(rw_stepwidth, rw_stddev_c, rw_limit_c),
  nrs0_c_(0.0, nr_stddev_c[0], g_rand.MakeSeed()),
  nrs1_c_(0.0, nr_stddev_c[1], g_rand.MakeSeed()),
  nrs2_c_(0.0, nr_stddev_c[2], g_rand.MakeSeed())
{

}

  void MagTorquer::MainRoutine(int count)
{
    // config
}

double MagTorquer::GetCurrent(int port_id) const
{
    if (!isOn_) return 0;
    return current_;
}

Vector<3> MagTorquer::activate(const Vector<3>& MagTorque_ordered, const libra::Vector<3>& MagEarth)
{
  // 指令磁気モーメントを機体座標系(B)からコンポーネント座標系(C)へ変換，q_b2cはセンサミスアライメントも含む．
  MagTorque_c = q_b2c_.frame_conv(MagTorque_ordered);
  //分解能　＋リミットチェック
  for (int i = 0; i < 3; ++i)
  {
    if (MagTorque_c[i] > max_c_[i])
    {
      MagTorque_c[i] = max_c_[i];
    }
    else if (MagTorque_c[i] < min_c_[i])
    {
      MagTorque_c[i] = min_c_[i];
    }
    else
    {
        // Specify the resolution in hexadecimal notation
      int ctrlDC = (int)((MagTorque_c[i] - min_c_[i]) / (max_c_[i] - min_c_[i])*resolution_);
      // Discretize the command torque
      MagTorque_c[i] = (max_c_[i] - min_c_[i]) / resolution_*ctrlDC + min_c_[i];
    }
    MagTorque_c[i] += bias_c_[i];//バイアス誤差
    MagTorque_c[i] += n_rw_c_[i];//ランダムウォーク

      // Gaussianノイズの加算
    if (i == 0)
    {
      MagTorque_c[0] += nrs0_c_;
    }
    else if (i == 1)
    {
      MagTorque_c[1] += nrs1_c_;
    }
    else if (i == 2)
    {
      MagTorque_c[2] += nrs2_c_;
    }

  }
  // ランダムウォークの更新
  ++n_rw_c_;

  // 機体座標系への変換
  MagTorque_b = q_c2b_.frame_conv(MagTorque_c);
  // 地球磁場を加味し，トルクに換算[Nm]
  Torque_b = outer_product(MagTorque_b, nT2T*MagEarth);
  return Torque_b;
}

Vector<3> MagTorquer::GetMagTorque_b()
{
  //MagTorque_bを取得．
  return MagTorque_b;
}

void MagTorquer::PrintParams(int sensor_id)
{
  cout << "magsensor" << sensor_id << "\n";
  cout << "q_b2c =(" << q_b2c_[0] << "," << q_b2c_[1] << "," << q_b2c_[2] << "," << q_b2c_[3] << ") \n";
  cout << "max =(" << max_c_[0] << "," << max_c_[1] << "," << max_c_[2] << ")  \n";
  cout << "min =(" << min_c_[0] << "," << min_c_[1] << "," << min_c_[2] << ")  \n";
  cout << "bias =(" << bias_c_[0] << "," << bias_c_[1] << "," << bias_c_[2] << ")  \n";
  cout << "resolution = " << resolution_ << "\n";
}

string MagTorquer::GetLogHeader() const
{
  string str_tmp = "";
  const string st_sensor_id = std::to_string(static_cast<long long>(sensor_id_));
  const char *cs = st_sensor_id.data();
  string MSSection = "mag_torquer";

  str_tmp += WriteVector(MSSection+cs, "b", "Am^2", 3);	//座標変換して地磁気ベクトルとの外積をとる前の値も入手する．
  str_tmp += WriteVector(MSSection + cs, "b", "Nm", 3);


  return str_tmp;
}

Vector<3> MagTorquer::GetMaxAm2() {
    return max_c_;
}

string MagTorquer::GetLogValue() const
{
  string str_tmp = "";
  str_tmp += WriteVector(MagTorque_b);
  str_tmp += WriteVector(Torque_b);

  return str_tmp;
}