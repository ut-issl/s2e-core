#pragma once

#include <Library/math/Quaternion.hpp>
#include <Library/math/Vector.hpp>
using namespace libra;

#include <Interface/LogOutput/ILoggable.h>

#include "PIDController.h"

const int MX = 0;
const int PX = 1;
const int MY = 2;
const int PY = 3;
const int MZ = 4;
const int PZ = 5;

class RVDController : public ILoggable {
 public:
  RVDController(double dt);
  ~RVDController();

  void SetTargetRelPosition(Vector<3> relPos_i);
  void SetTargetRelVelocity(Vector<3> relVel_i);
  void SetTargetRelAttitude(Quaternion q12);

  void SetPositionGain(double p, double d, double i);

  Vector<2> CalcCurrent(Vector<3> relpos_now_i, Quaternion q1_ib, Quaternion q2_ib);

  // 3軸±方向に必要なスラスト
  Vector<6> CalcThrust(Vector<3> relpos_now_i, Quaternion q_i2b);
  Vector<6> CalcThrust(Vector<3> relpos_now_i, Vector<3> relvel_now_i, Quaternion q_i2b);
  Vector<6> CalcThrustVeloc(Vector<3> relvel_now_i, Quaternion q_i2b);

  // 3軸方向に必要なトルク
  Vector<3> CalcRW(Quaternion q_ib_now);

  inline Vector<3> GetForce_b() { return thrust_b_; }
  inline Vector<3> GetTorque() { return torque_; }
  inline void ClearForce() { thrust_b_ *= 0; }

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  double acos_tolerant(double x);
  Vector<3> thrust_b_;
  Vector<3> torque_;

  PIDController<3> relPosCtrl;
  PIDController<3> relAttCtrl;
  PIDController<3> relVelCtrl;

  // 目標相対位置（母機から見た子機の慣性系座標）
  Vector<3> tar_relpos_i;

  // 目標相対位置（母機から見た子機の慣性系座標）
  Vector<3> tar_relvel_i;

  // 目標クオータニオン
  Quaternion tar_q_i2b;

  Vector<6> CalcThrustEach(Vector<3> thrusts);
};
