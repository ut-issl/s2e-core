#ifndef __SunSensor_H__
#define __SunSensor_H__

#include "../../Environment/Local/SRPEnvironment.h"
#include "../../Library/math/Vector.hpp"
using libra::Vector;
#include "../../Library/math/Quaternion.hpp"
using libra::Quaternion;
#include "../../Interface/LogOutput/ILoggable.h"
#include "../Abstract/ComponentBase.h"

class SunSensor: public ComponentBase, public ILoggable
{
public:
  Quaternion q_b2c_;
  Vector<3> sun_c_;
  Vector<3> measured_sun_c_;
  double alpha_; //太陽方向ベクトル(sc2sun)をセンサ座標系xz平面に射影したベクトルのz軸から測った角度[rad]
  double beta_; //太陽方向ベクトル(sc2sun)をセンサ座標系yz平面に射影したベクトルのz軸から測った角度[rad]
  double detectable_angle_rad_;
  double sun_angle_;
  bool sun_detected_flag_;
  double ss_wnvar_;
  double ss_bivar_;
  double ss_bias_;
  const SRPEnvironment *srp_;
  void SunDetectionJudgement(bool sun_eclipsed);

  SunSensor(
    const libra::Quaternion& q_b2c, 
    double detectable_angle_rad,
    double ss_wnvar,
    double ss_bivar,
    const SRPEnvironment *srp);

  void MainRoutine(int time_count) override;
  void measure(const libra::Vector<3>& sun_b, bool sun_eclipsed);

  bool GetSunDetected();
  Vector<3> GetMeasuredSun_c();
  Vector<3> GetMeasuredSun_b();
  double GetSunAngleAlpha();
  double GetSunAngleBeta();
  void SetTanRange(double);

  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;
};

#endif //__SunSensor_H__