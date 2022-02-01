#ifndef __SunSensor_H__
#define __SunSensor_H__

#include <Environment/Local/SRPEnvironment.h>
#include <Library/math/Vector.hpp>
#include <Library/math/NormalRand.hpp>
#include <Library/math/Quaternion.hpp>
#include <Interface/LogOutput/ILoggable.h>
#include "../Abstract/ComponentBase.h"

class SunSensor: public ComponentBase, public ILoggable
{
public:
  SunSensor(
    const int prescaler,
    ClockGenerator* clock_gen,
    const int id,
    const libra::Quaternion& q_b2c, 
    const double detectable_angle_rad,
    const double nr_stddev_c,
    const double nr_bias_stddev_c,
    const double intensity_lower_threshold_percent,
    const SRPEnvironment *srp
  );
  SunSensor(
    const int prescaler,
    ClockGenerator* clock_gen,
    PowerPort* power_port,
    const int id,
    const libra::Quaternion& q_b2c, 
    const double detectable_angle_rad,
    const double nr_stddev_c,
    const double nr_bias_stddev_c,
    const double intensity_lower_threshold_percent,
    const SRPEnvironment *srp
  );

  //ComponentBase override function
  void MainRoutine(int count) override;
  // ILogabble override functions
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;
  // Getter
  inline const bool GetSunDetectedFlag() const { return sun_detected_flag_; };
  inline const Vector<3> GetMeasuredSun_c() const { return measured_sun_c_; };
  inline const Vector<3> GetMeasuredSun_b() const { return q_b2c_.conjugate().frame_conv(measured_sun_c_); };
  inline const double GetSunAngleAlpha() const { return alpha_; };
  inline const double GetSunAngleBeta() const { return beta_; };
  inline const double GetSolarIlluminance() const { return solar_illuminance_; };

protected:
  const int id_;
  libra::Quaternion q_b2c_;  // Quaternion from body frame to component frame (Z-axis of the component is sight direction)
  double intensity_lower_threshold_percent_; // If the light intensity becomes smaller than this, it becomes impossible to get the sun direction

  libra::Vector<3> sun_c_{0.0};
  libra::Vector<3> measured_sun_c_{0.0};
  double alpha_=0.0;  // Angle between Z-axis and the sun direction projected on XZ plane [rad]
  double beta_=0.0;   // Angle between Z-axis and the sun direction projected on YZ plane [rad]
  double solar_illuminance_ = 0.0;// The energy of sunlight per unit area, taking into account the angle to the sun[W/m^2].

  double detectable_angle_rad_; //half angle (>0) [rad]
  bool sun_detected_flag_ = false;
  // Noise parameters
  libra::NormalRand nrs_alpha_; // Normal random
  libra::NormalRand nrs_beta_; // Normal random
  double bias_alpha_=0.0; // Normal random for bias
  double bias_beta_=0.0; // Normal random for bias

  // Measured variables
  const SRPEnvironment *srp_;
 
  // functions
  void SunDetectionJudgement();
  void SunDetectionJudgement(bool sun_eclipsed); // This function is old version, but retained for backward compatibility
  void measure(const libra::Vector<3>& sun_b);
  void measure(const libra::Vector<3>& sun_b, bool sun_eclipsed); // This function is old version, but retained for backward compatibility
  double TanRange(double x);
  void Initialize(const double nr_stddev_c,const double nr_bias_stddev_c);
  void CalcSolarIlluminance();
};

#endif //__SunSensor_H__
