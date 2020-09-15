/*!
  \file   MagSensor.hpp
  \author KUDO, Takumi.
  \date
  \brief  MagSensorモデル, ST5.hppを改造
*/
#ifndef MagSensor_H_
#define MagSensor_H_

#include <cstring>
#define _CRT_SECURE_NO_WARNINGS
#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/Vector.hpp"
#include "../../Library/math/Matrix.hpp"
#include "../../Library/math/NormalRand.hpp"
#include "../../Library/math/RandomWalk.hpp"
#include "../../Interface/LogOutput/ILoggable.h"
#include "../Abstract/ComponentBase.h"
#include "../../Environment/Local/LocalEnvironment.h"

using namespace std;

class MagSensor: public ComponentBase, public ILoggable
{
public:
  //! コンストラクタ
  /*!
  \q_b2c : 機体座標系(B)→センサ座標系(C)変換Quaternion
  \ScaleFactor : スケールファクタ(3×3行列)
  \MisAlign : センサミスアライメント(3×3行列)
  \param Bias バイアス定常成分(3次元ベクトル)[nT]
  \rw_stepwidth : ODE<3>に渡すステップ幅[s]
  \param rw_stddev ランダムウォーク標準偏差[nT]
  \param rw_limit ランダムウォーク制限値[nT]
  \param nr_stddev ガウスノイズ標準偏差[nT]
  */
  MagSensor(ClockGenerator* clock_gen,
    const int sensor_id,
    const libra::Quaternion& q_b2c,
    const libra::Matrix<3, 3>& scale_factor,	//スケールファクタ，磁場に対する出力電圧の比[V/Gauss], 1[mGauss] = 100[nT]．対角行列での使用を想定．
    const libra::Vector<3>& bias_c,			    //零点誤差，外部磁場がない時のオフセット[nT]
    double rw_stepwidth,                       //ODEのステップ幅（のはず）．ODE<3>にdoubleで引数を渡しているので，ここはdoubleでなければならない．
    const libra::Vector<3>& rw_stddev_c,
    const libra::Vector<3>& rw_limit_c,
    const libra::Vector<3>& nr_stddev_c,
	  const MagEnvironment *magnet);

  libra::Vector<3> mag_c;
  libra::Vector<3> measure(const libra::Vector<3>& mag_b);
  void MainRoutine(int count);
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;
  void PrintParams(int sensor_id);  //デバッグ出力
private:
  //! 機体座標系(B)→センサ座標系(C)変換Quaternion
  libra::Quaternion q_b2c_;
  //! スケールファクタ
  libra::Matrix<3, 3> scale_factor_;
  //! バイアス定常成分
  libra::Vector<3> bias_c_;
  //! ランダムウォーク生成オブジェクト
  RandomWalk n_rw_c_;
  //! 正規乱数生成オブジェクト
  libra::NormalRand nrs0_c_, nrs1_c_, nrs2_c_;
  // センサーID
  const int sensor_id_;
  const MagEnvironment* magnet_;
};

#endif 