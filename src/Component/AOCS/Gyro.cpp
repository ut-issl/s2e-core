/*!
\file   MagSensor.cpp
\author KUDO Takumi
\date   Sat May 14 14:14:23 2011
\brief  Gyro.cppの実装
*/
#include "Gyro.h"
#include "../../Interface/SpacecraftInOut/SpacecraftInterface.h"
#include "../../Interface/SpacecraftInOut/SCIDriver.h"
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
  SCIDriver::ConnectPort(port_id);
}

Gyro::~Gyro()
{
  SCIDriver::ClosePort(port_id_);
}

void Gyro::MainRoutine(int count)
{
  ReceiveCommand();
  measure(dynamics_->GetAttitude().GetOmega_b());
  SendTelemetry();
}

double Gyro::GetCurrent(int port_id) const
{
  if (!isOn_) return 0;
  return current_;
}

int Gyro::ReceiveCommand()
{
  unsigned char rxb[RX_SIZE];
  int ret = SCIDriver::ReceiveFromSC(port_id_, rxb, 0, RX_SIZE);
  if (ret == 0) return 0;
  memcpy(rx_buff_, rxb, RX_SIZE);
  ParseCommand(rx_buff_);
  return 0;
}

int Gyro::ParseCommand(unsigned char * cmd)
{
  //FIXME: use strlen not sizeof
  if (sizeof(cmd) != RX_SIZE)
  {
    return -1;
  }
  if (cmd[0] != 'G' || cmd[1] != 'C' || cmd[4] != 0x50) //ヘッダフッタ処理
  {
    return -1;
  }
  if (cmd[2] == 0x30) //CMD ID 0x30でstatusを変更する
  {
    status = cmd[3];
  }

  return 0;
}

int Gyro::SendTelemetry()
{
  // テレメ生成
  unsigned char header[] = "$TSC,BIN,";
  uint16_t data_size = 0x1c00;
  uint32_t counter = 40;  //一先ず試験のため固定値を入れる
  float omega_f[3];
  double omega_LSB = 10.0 / pow(2.0, 31);
  for (int i=0;i<3;i++) omega_f[i] = quantization_f(omega_c[i], omega_LSB);
  uint16_t sld_current[] = {10,20,30};  //一先ず試験のため固定値を入れる
  uint16_t temperature = 100; //一先ず試験のため固定値を入れる
  unsigned char check_sum_boundary[] = "*";
  uint16_t footer = 0x0d0a;

  int indx = 0;
  memcpy(tx_buff_+indx, header, 9);
  indx += 9;
  memcpy(tx_buff_ + indx, &data_size, 2);
  indx += 2;
  memcpy(tx_buff_ + indx, &counter, 4);
  indx += 4;
  memcpy(tx_buff_ + indx, &status, 2);
  indx += 2;
  memcpy(tx_buff_ + indx, &omega_f[0], 4);
  indx += 4;
  memcpy(tx_buff_ + indx, &omega_f[1], 4);
  indx += 4;
  memcpy(tx_buff_ + indx, &omega_f[2], 4);
  indx += 4;
  memcpy(tx_buff_ + indx, &sld_current[0], 2);
  indx += 2;
  memcpy(tx_buff_ + indx, &sld_current[1], 2);
  indx += 2;
  memcpy(tx_buff_ + indx, &sld_current[2], 2);
  indx += 2;
  memcpy(tx_buff_ + indx, &temperature, 2);
  indx += 2;
  memcpy(tx_buff_ + indx, &check_sum_boundary, 1);
  indx += 1;
  //チェックサム計算
  uint8_t check_sum=0;
  for (int i=1;i<TX_SIZE - 4;i++)
  {
    check_sum = check_sum ^ tx_buff_[i];
  }
  unsigned char check_sum_ascii[2];
  check_sum_ascii[0] = (check_sum & 0xf0)>>4;
  check_sum_ascii[1] = (check_sum & 0x0f);
  for (int i=0;i<2;i++)
  {
    if(check_sum_ascii[i] < 10)
    {
      check_sum_ascii[i] += 0x30;
    }
    else
    {
      check_sum_ascii[i] += (-10+0x65);
    }
  }
  memcpy(tx_buff_ + indx, &check_sum_ascii,2);
  indx += 2;
  memcpy(tx_buff_ + indx, &footer, 2);
  indx += 2;

  //送信
  SCIDriver::SendToSC(port_id_, tx_buff_, 0, TX_SIZE);

  return 0;
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
