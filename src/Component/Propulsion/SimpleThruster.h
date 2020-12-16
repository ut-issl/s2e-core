#pragma once

#include "../../Library/math/NormalRand.hpp"
using libra::NormalRand;
#include "../../Library/math/Vector.hpp"
using libra::Vector;
#include "../../Library/math/Quaternion.hpp"
using libra::Quaternion;

#include "../Abstract/ComponentBase.h"
#include "../../Dynamics/Dynamics.h"
#include "../../Simulation/Spacecraft/Structure/Structure.h"
#include "../../Interface/LogOutput/Logger.h"

class SimpleThruster : public ComponentBase, public ILoggable
{
public:
  SimpleThruster(ClockGenerator* clock_gen, Vector<3> thruster_pos, Vector<3> thruster_dir, double max_mag, double mag_err, double deg_err, int id, const Structure* structure, const Dynamics* dynamics);
  //コンストラクタ
  ~SimpleThruster();
  void MainRoutine(int count);
  void SetDuty(double dutyratio);

  // 推力の計算 
  // isReal: 真なら実際の値を出力、偽なら理論上の値（ノイズなし）を出力
  void CalcThrust(bool isReal = true);
  void CalcTorque(Vector<3> center, double temp);

  Vector<3> GetThrust();
  Vector<3> GetTorque();

  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

private:
  //内部状態
  double duty_; // スラスタバルブ開閉のデューティ比
                //基本情報
  Vector<3> thruster_pos_;  //スラスタ取り付け位置
  Vector<3> thrust_dir_;//スラスト方向ベクトル
  double thrust_magnitude_max;
  double thrust_magnitude_err_; //スラスト大きさ揺らぎ //時間独立のランダム変数でとりあえずよい
  double thrust_dir_err_;//スラスト方向揺らぎの誤差角
  NormalRand mag_nr, dir_nr;
  Vector<3> thrust_b_;
  Vector<3> torque_b_;
  const int id_;

  double CalcThrustMagnitude();
  Vector<3> CalcThrustDir();

  const Structure* structure_;
  const Dynamics* dynamics_;
};
