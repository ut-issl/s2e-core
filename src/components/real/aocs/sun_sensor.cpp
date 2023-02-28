/*
 * @file sun_sensor.cpp
 * @brief Class to emulate sun sensor
 */

#include "sun_sensor.hpp"

#include <library/math/constants.hpp>
#include <library/randomization/normal_randomization.hpp>
using libra::NormalRand;
#include <library/logger/log_utility.hpp>
#include <library/randomization/global_randomization.hpp>

using namespace std;

SunSensor::SunSensor(const int prescaler, ClockGenerator* clock_generator, const int component_id, const libra::Quaternion& quaternion_b2c,
                     const double detectable_angle_rad, const double normal_random_standard_deviation_c_Am2, const double nr_bias_stddev_c,
                     const double intensity_lower_threshold_percent, const SolarRadiationPressureEnvironment* srp,
                     const LocalCelestialInformation* local_celestial_information)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      intensity_lower_threshold_percent_(intensity_lower_threshold_percent),
      detectable_angle_rad_(detectable_angle_rad),
      srp_(srp),
      local_celestial_information_(local_celestial_information) {
  Initialize(normal_random_standard_deviation_c_Am2, nr_bias_stddev_c);
}

SunSensor::SunSensor(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                     const libra::Quaternion& quaternion_b2c, const double detectable_angle_rad, const double normal_random_standard_deviation_c_Am2,
                     const double nr_bias_stddev_c, const double intensity_lower_threshold_percent, const SolarRadiationPressureEnvironment* srp,
                     const LocalCelestialInformation* local_celestial_information)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      intensity_lower_threshold_percent_(intensity_lower_threshold_percent),
      detectable_angle_rad_(detectable_angle_rad),
      srp_(srp),
      local_celestial_information_(local_celestial_information) {
  Initialize(normal_random_standard_deviation_c_Am2, nr_bias_stddev_c);
}

void SunSensor::Initialize(const double normal_random_standard_deviation_c_Am2, const double nr_bias_stddev_c) {
  // Bias
  NormalRand nr(0.0, nr_bias_stddev_c, global_randomization.MakeSeed());
  bias_noise_alpha_rad_ += nr;
  bias_noise_beta_rad_ += nr;

  // Normal Random
  random_noise_alpha_.SetParameters(0.0, normal_random_standard_deviation_c_Am2);  // global_randomization.MakeSeed()
  random_noise_beta_.SetParameters(0.0, normal_random_standard_deviation_c_Am2);   // global_randomization.MakeSeed()
}
void SunSensor::MainRoutine(int count) {
  UNUSED(count);

  measure();
}

void SunSensor::measure() {
  libra::Vector<3> sun_pos_b = local_celestial_information_->GetPositionFromSpacecraft_b_m("SUN");
  libra::Vector<3> sun_dir_b = Normalize(sun_pos_b);

  sun_direction_true_c_ = quaternion_b2c_.FrameConversion(sun_dir_b);  // Frame conversion from body to component

  SunDetectionJudgement();  // Judge the sun is inside the FoV

  if (sun_detected_flag_) {
    alpha_rad_ = atan2(sun_direction_true_c_[0], sun_direction_true_c_[2]);
    beta_rad_ = atan2(sun_direction_true_c_[1], sun_direction_true_c_[2]);
    // Add constant bias noise
    alpha_rad_ += bias_noise_alpha_rad_;
    beta_rad_ += bias_noise_beta_rad_;

    // Add Normal random noise
    alpha_rad_ += random_noise_alpha_;
    beta_rad_ += random_noise_beta_;

    // Range [-π/2:π/2]
    alpha_rad_ = TanRange(alpha_rad_);
    beta_rad_ = TanRange(beta_rad_);

    measured_sun_direction_c_[0] = tan(alpha_rad_);
    measured_sun_direction_c_[1] = tan(beta_rad_);
    measured_sun_direction_c_[2] = 1.0;

    measured_sun_direction_c_ = Normalize(measured_sun_direction_c_);
  } else {
    measured_sun_direction_c_ = libra::Vector<3>(0);
    alpha_rad_ = 0.0;
    beta_rad_ = 0.0;
  }

  CalcSolarIlluminance();
}

void SunSensor::SunDetectionJudgement() {
  libra::Vector<3> sun_direction_c = Normalize(sun_direction_true_c_);

  double sun_angle_ = acos(sun_direction_c[2]);

  if (solar_illuminance_W_m2_ < intensity_lower_threshold_percent_ / 100.0 * srp_->GetSolarConstant_W_m2()) {
    sun_detected_flag_ = false;
  } else {
    if (sun_angle_ < detectable_angle_rad_) {
      sun_detected_flag_ = true;
    } else {
      sun_detected_flag_ = false;
    }
  }
}

void SunSensor::CalcSolarIlluminance() {
  libra::Vector<3> sun_direction_c = Normalize(sun_direction_true_c_);
  double sun_angle_ = acos(sun_direction_c[2]);

  if (sun_angle_ > libra::pi_2) {
    solar_illuminance_W_m2_ = 0.0;
    return;
  }

  double power_density = srp_->GetPowerDensity_W_m2();
  solar_illuminance_W_m2_ = power_density * cos(sun_angle_);
  // TODO: Take into account the effects of albedo.
}

double SunSensor::TanRange(double x) {
  if (x > libra::pi_2) x = libra::pi - x;
  if (x < -libra::pi_2) x = -libra::pi - x;
  return x;
}

string SunSensor::GetLogHeader() const {
  string str_tmp = "";
  const string sensor_id = std::to_string(static_cast<long long>(component_id_));
  std::string sensor_name = "sun_sensor" + sensor_id + "_";

  str_tmp += WriteVector(sensor_name + "measured_sun_direction", "c", "-", 3);
  str_tmp += WriteScalar(sensor_name + "sun_detected_flag", "-");

  return str_tmp;
}

string SunSensor::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(measured_sun_direction_c_);
  str_tmp += WriteScalar(double(sun_detected_flag_));

  return str_tmp;
}
