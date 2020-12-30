/*!
\file   GYRO.h
\author KUDO, Takumi.
\date
\brief  Gyroモデル．
*/
#ifndef Gyro_H_
#define Gyro_H_

#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/Vector.hpp"
#include "../../Library/math/Matrix.hpp"
#include "../../Library/math/NormalRand.hpp"
#include "../../Library/math/Quantization.h"
#include "../../Interface/LogOutput/ILoggable.h"
#include "../../Library/math/RandomWalk.hpp"
#include "../Abstract/ComponentBase.h"
#include "../../Dynamics/Dynamics.h"
using namespace std;

class Gyro : public ComponentBase, public ILoggable
{
public:
  //! コンストラクタ
  /*!
  \q_b2c : 機体座標系(B)→センサ座標系(C)変換Quaternion
  \port_id : OBCとの通信用ポートID
  \ScaleFactor : スケールファクタ(3×3行列)
  \MisAlign : センサミスアライメント(3×3行列)
  \Bias バイアス定常成分(3次元ベクトル)[rad/s]
  \rw_stepwidth : ODE<3>に渡すステップ幅[s]
  \rw_stddev ランダムウォーク標準偏差[rad/s]
  \rw_limit ランダムウォーク制限値[rad/s]
  \nr_stddev ガウスノイズ標準偏差[rad/s]
  \range_to_const_c コンポ座標系において角速度の大きさがこの角速度を超えると，Gyro出力は一定となる．
  \range_to_zero_c コンポ座標系において角速度の大きさがこの角速度を超えると，Gyro出力は0となる．
  \current 消費電流値
  \dynamics measureで参照するためのdynamics
  */
  Gyro(
    ClockGenerator* clock_gen,
    const int sensor_id,
    const int port_id,
    const libra::Quaternion& q_b2c,
    const libra::Matrix<3, 3>& scale_factor,
    const libra::Vector<3>& bias_c,
    double rw_stepwidth,
    const libra::Vector<3>& rw_stddev_c,
    const libra::Vector<3>& rw_limit_c,
    const libra::Vector<3>& nr_stddev_c,
    double range_to_const_c,
    double range_to_zero_c,
    double current,
    const Dynamics *dynamics
  );								

  ~Gyro();

  void MainRoutine(int count);
  double GetCurrent(int port_id) const;
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

  libra::Vector<3> measure(const libra::Vector<3>& omega_b); //本当はprivateにしたいがXACTが使っているので
  double GetRangeToConst(); //本当はprivateにしたいがXACTが使っているので
  void PrintParams(int sensor_id);  //デバッグ出力

  //Getter
  inline const libra::Vector<3>& GetOmegaC(void)const{return omega_c;}

private:
  void RangeCheck(void);										//レンジが両方正であり，かつrange_to_const_c < range_to_zero_cを満たしていることをチェック．満たしていないとinvalid_argumentが出る．
  void Clip(void);											//コンポ系角速度とレンジを比較し，Gyro出力を返す．

  libra::Vector<3> omega_c{0.0};
  uint16_t status = 0;
  libra::Quaternion q_b2c_;//! 機体座標系(B)→センサ座標系(C)変換Quaternion
  //! スケールファクタ
  libra::Matrix<3, 3> scale_factor_;
  //! バイアス定常成分
  libra::Vector<3> bias_c_;
  //! ランダムウォーク生成オブジェクト
  RandomWalk n_rw_c_;
  //! 正規乱数生成オブジェクト
  libra::NormalRand nrs0_c_, nrs1_c_, nrs2_c_;
  double range_to_const_c_;
  double range_to_zero_c_;
  //電流値　A
  double current_;
  // センサーID
  const int sensor_id_;
  const int port_id_;
  //送信バッファ
  const static int TX_SIZE = 42; //Byte
  unsigned char tx_buff_[TX_SIZE];
  const static int RX_SIZE = 5; //Byte
  unsigned char rx_buff_[RX_SIZE];

  const Dynamics* dynamics_;
};

#endif 