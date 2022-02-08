#include "SunSensor.h"
#include <Library/math/NormalRand.hpp>
using libra::NormalRand;
#include <Library/math/GlobalRand.h>
#include <Interface/LogOutput/LogUtility.h>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

SunSensor::SunSensor(
  const int prescaler,
  ClockGenerator* clock_gen,
  const int id,
  const libra::Quaternion& q_b2c, 
  const double detectable_angle_rad,
  const double nr_stddev_c,
  const double nr_bias_stddev_c,
  const double intensity_lower_threshold_percent,
  const SRPEnvironment* srp,
  const LocalCelestialInformation* local_celes_info)
  : ComponentBase(prescaler,clock_gen), id_(id),
    q_b2c_(q_b2c), detectable_angle_rad_(detectable_angle_rad), intensity_lower_threshold_percent_(intensity_lower_threshold_percent),
    srp_(srp), local_celes_info_(local_celes_info)
{
  Initialize(nr_stddev_c,nr_bias_stddev_c);
}

SunSensor::SunSensor(
  const int prescaler,
  ClockGenerator* clock_gen,
  PowerPort* power_port,
  const int id,
  const libra::Quaternion& q_b2c, 
  const double detectable_angle_rad,
  const double nr_stddev_c,
  const double nr_bias_stddev_c,
  const double intensity_lower_threshold_percent,
  const SRPEnvironment* srp,
  const LocalCelestialInformation* local_celes_info)
  : ComponentBase(prescaler,clock_gen), id_(id),
    q_b2c_(q_b2c), detectable_angle_rad_(detectable_angle_rad), intensity_lower_threshold_percent_(intensity_lower_threshold_percent),
    srp_(srp), local_celes_info_(local_celes_info)
{
  Initialize(nr_stddev_c,nr_bias_stddev_c);
}

void SunSensor::Initialize(const double nr_stddev_c,const double nr_bias_stddev_c)
{
  // Bias
  NormalRand nr(0.0, nr_bias_stddev_c, g_rand.MakeSeed());
  bias_alpha_ += nr;
  bias_beta_ += nr;

  // Normal Random
  nrs_alpha_.set_param(0.0, nr_stddev_c);  //g_rand.MakeSeed()
  nrs_beta_.set_param(0.0, nr_stddev_c);  //g_rand.MakeSeed()
}
void SunSensor::MainRoutine(int count)
{
  Vector<3> sun_pos_b = local_celes_info_->GetPosFromSC_b("SUN");
  Vector<3> sun_dir_b = normalize(sun_pos_b);
  measure(sun_dir_b);
}

void SunSensor::measure(const Vector<3>& sun_b)
{
  sun_c_ = q_b2c_.frame_conv(sun_b);    // Frame conversion from body to component

  SunDetectionJudgement();  // Judge the sun is inside the FoV

  if (sun_detected_flag_)
  {
    alpha_ = atan2(sun_c_[0], sun_c_[2]);
    beta_ = atan2(sun_c_[1], sun_c_[2]);
    // Add constant bias noise
    alpha_ += bias_alpha_;
    beta_ += bias_beta_;

    // Add Normal random noise
    alpha_ += nrs_alpha_;
    beta_ += nrs_beta_;

    // Range [-π/2:π/2]
    alpha_ = TanRange(alpha_);
    beta_ = TanRange(beta_);

    measured_sun_c_[0] = tan(alpha_);
    measured_sun_c_[1] = tan(beta_);
    measured_sun_c_[2] = 1.0;

    measured_sun_c_ = normalize(measured_sun_c_);
  }
  else
  {
    measured_sun_c_ = Vector<3>(0);
    alpha_ = 0.0;
    beta_ = 0.0;
  }

  CalcSolarIlluminance();
}

void SunSensor::measure(const Vector<3>& sun_b, bool is_eclipsed)
{
  sun_c_ = q_b2c_.frame_conv(sun_b);    // Frame conversion from body to component

  SunDetectionJudgement(is_eclipsed);  // Judge the sun is inside the FoV

  if (sun_detected_flag_)
  {
    alpha_ = atan2(sun_c_[0], sun_c_[2]);
    beta_ = atan2(sun_c_[1], sun_c_[2]);
    // Add constant bias noise
    alpha_ += bias_alpha_;
    beta_ += bias_beta_;

    // Add Normal random noise
    alpha_ += nrs_alpha_;
    beta_ += nrs_beta_;

    // Range [-π/2:π/2]
    alpha_ = TanRange(alpha_);
    beta_ = TanRange(beta_);

    measured_sun_c_[0] = tan(alpha_);
    measured_sun_c_[1] = tan(beta_);
    measured_sun_c_[2] = 1.0;

    measured_sun_c_ = normalize(measured_sun_c_);
  }
  else
  {
    measured_sun_c_ = Vector<3>(0);
    alpha_ = 0.0;
    beta_ = 0.0;
  }

  CalcSolarIlluminance();
}

void SunSensor::SunDetectionJudgement()
{
  Vector<3> sun_direction_c = normalize(sun_c_);

  double sun_angle_ = acos(sun_direction_c[2]);

  if (solar_illuminance_ < intensity_lower_threshold_percent_ / 100.0 * srp_->GetSolarConstant())
  {
    sun_detected_flag_ = false;
  }
  else{
    if (sun_angle_ < detectable_angle_rad_)
    {
      sun_detected_flag_ = true;
    }
    else{
      sun_detected_flag_ = false;
    }
  }
}

void SunSensor::SunDetectionJudgement(bool sun_eclipsed)
{
  Vector<3> sun_direction_c = normalize(sun_c_);

  double sun_angle_ = acos(sun_direction_c[2]);

  if (sun_eclipsed)
  {
    sun_detected_flag_ = false;
  }
  else {
    if (sun_angle_ < detectable_angle_rad_)
    {
      sun_detected_flag_ = true;
    }
    else {
      sun_detected_flag_ = false;
    }
  }
}

void SunSensor::CalcSolarIlluminance()
{
  Vector<3> sun_direction_c = normalize(sun_c_);
  double sun_angle_ = acos(sun_direction_c[2]);

  if (sun_angle_ > M_PI_2)
  {
    solar_illuminance_ = 0.0;
    return;
  }

  double power_density = srp_->CalcPowerDensity();
  solar_illuminance_ = power_density * cos(sun_angle_);
  //TODO: Take into account the effects of albedo.
}

double SunSensor::TanRange(double x)
{
  if (x>  M_PI / 2.0) x = M_PI - x;
  if (x< -M_PI / 2.0) x = -M_PI - x;
  return x;
}

string SunSensor::GetLogHeader() const
{
  string str_tmp = "";
  const string st_id = std::to_string(static_cast<long long>(id_));

  str_tmp += WriteVector("sun"+st_id, "c", "-", 3);
  str_tmp += WriteScalar("sun_detected_flag"+st_id,"-");

  return str_tmp;
}

string SunSensor::GetLogValue() const
{
  string str_tmp = "";

  str_tmp += WriteVector(measured_sun_c_);
  str_tmp += WriteScalar(double(sun_detected_flag_));

  return str_tmp;
}
