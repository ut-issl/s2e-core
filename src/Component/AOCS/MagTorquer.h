#ifndef MTQ_H_
#define MTQ_H_

#include <cstring>
#define _CRT_SECURE_NO_WARNINGS
#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/Vector.hpp"
#include "../../Library/math/Matrix.hpp"
#include "../../Library/math/NormalRand.hpp"
#include "../../Library/math/RandomWalk.hpp"
#include "../Abstract/ComponentBase.h"
#include "../../Interface/LogOutput/ILoggable.h"

using namespace std;

class MagTorquer: public ComponentBase, public ILoggable
{
public:
  //! コンストラクタ
  /*!
    \q_b2c : 機体座標系(B)→コンポ座標系(C)変換Quarternion センサミスアライメントも含む．
  \const libra::Vector<3>& max, 指令磁気モーメント最大値
  \const libra::Vector<3>& min, 指令磁気モーメント最小値
    \param Bias バイアス定常成分(3次元ベクトル)
  \rw_stepwidth : ODE<3>に渡すステップ幅
    \param rw_stddev ランダムウォーク標準偏差
    \param rw_limit ランダムウォーク制限値
    \param nr_stddev ガウスノイズ標準偏差
  \c_mtq 磁気トルカ発生磁気モーメント[Am^2]
  \MagEarth 地球磁場[T] 座標系は機体座標系(B)
  \resoluion 解像度
  */
  MagTorquer(ClockGenerator* clock_gen,
    const int sensor_id,
    const libra::Quaternion& q_b2c,
    const libra::Matrix<3, 3>& scale_facter,
    const libra::Vector<3>& max_c,
    const libra::Vector<3>& min_c,
    const libra::Vector<3>& bias_c,
    double rw_stepwidth,                       //ODEのステップ幅（のはず）．ODE<3>にdoubleで引数を渡しているので，ここはdoubleでなければならない．
    const libra::Vector<3>& rw_stddev_c,
    const libra::Vector<3>& rw_limit_c,
    const libra::Vector<3>& nr_stddev_c,
    unsigned int resolution);

  //指令トルク(期待座標系(B))，地球磁場(機体座標系(B))を入力し，磁気トルカの出力を機体座標系(B)で返す．
  libra::Vector<3> activate(const libra::Vector<3>& c_mtq, const libra::Vector<3>& MagEarth);
  void MainRoutine(int count);
  double GetCurrent(int port_id) const;

  void PrintParams(int sensor_id);  //デバッグ出力
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

  Vector<3> GetMaxAm2();

  libra::Vector<3> Torque_b;
  libra::Vector<3> MagTorque_c;
  libra::Vector<3> MagTorque_b;
  libra::Vector<3> GetMagTorque_b();

private:
  //! 機体座標系(B)→ コンポ座標系(C)変換Quaternion
  libra::Quaternion q_b2c_;
  //! コンポ座標系(C)→機体座標系(B)変換Quaternion
  libra::Quaternion q_c2b_;
  //
  unsigned int resolution_;
  double zero_limit_;
  double current_;
  libra::Vector<3> max_c_;
  libra::Vector<3> min_c_;
  //! スケールファクタ
  libra::Matrix<3, 3> scale_factor_;
  //! バイアス定常成分
  libra::Vector<3> bias_c_;
  //! ランダムウォーク生成オブジェクト
  RandomWalk n_rw_c_;
  //! 正規乱数生成オブジェクト
  libra::NormalRand nrs0_c_, nrs1_c_, nrs2_c_;
  //! nT→Tに変換する定数
  const double nT2T = 1.0e-9;
  // センサーID
  const int sensor_id_;

};

#endif // MTQ_H_
