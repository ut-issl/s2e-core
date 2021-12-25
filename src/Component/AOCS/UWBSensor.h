#pragma once
#include "../../Library/math/Vector.hpp"
#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/NormalRand.hpp"
using libra::Vector;
using libra::Quaternion;
using libra::NormalRand;

#include "../../Interface/LogOutput/ILoggable.h"

class UWBSensor
{
public:
  UWBSensor(int sensor_id, Vector<3> pos_b_, Vector<3> dir_b_, Vector<3> axis_b_);
  ~UWBSensor();

  // 対向UWBの電波送受信レベルが十分高ければ1を、受信できない状況では0を返す
  // otherはターゲット側のUWBセンサーとする
  int IsVisible(UWBSensor& other);

  Vector<3> LocationTo(UWBSensor& other);

  double MeasureDistanceTo(UWBSensor& other);

  // pos_i: 慣性系における宇宙機の位置ベクトル, q_i2b: 宇宙機のクオータニオン
  void SetParameters(Vector<3> pos_i, Quaternion q_i2b);

  inline Vector<3> GetPos_b() { return pos_b; }

private:
  // 光速 [m/s]
  static const int c = 299792458;

  // 受信限界ゲイン [dBm]
  static const int Plimit = -105;

  const int sensor_id;

  // 機体座標でのUWBセンサーの位置ベクトル
  Vector<3> pos_b;

  // UWB平面の法線ベクトル
  Vector<3> dir_b;

  // DPアンテナの軸方向
  Vector<3> axis_b;

  // 取り付けている宇宙機の慣性座標での位置
  Vector<3> ref_pos_i;

  // 慣性座標から取り付けている宇宙機の機体座標へのクオータニオン
  Quaternion q_b2i;

  // 送信電力[dBm]
  double Pt = 9.3;

  // 中心周波数 [Hz]
  double fc = 3993.6e6;
  // 測距の観測ノイズ源
  NormalRand measure_nr;

  // theta方向のアンテナゲインを計算する
  double CalcAntennaGain(double theta);

  // 測距ノイズの偏差[m]（距離に依存する）を返す
  double CalcDeviation(double distance);
};
