/*
 * @file sun_sensor.cpp
 * @brief Class to emulate sun sensor
 */

#include "sun_sensor.hpp"

#include <math_physics/math/constants.hpp>
#include <math_physics/randomization/normal_randomization.hpp>
using randomization::NormalRand;
#include <logger/log_utility.hpp>
#include <math_physics/randomization/global_randomization.hpp>
#include <setting_file_reader/initialize_file_access.hpp>

using namespace std;

SunSensor::SunSensor(const int prescaler, ClockGenerator* clock_generator, const int component_id, const math::Quaternion& quaternion_b2c,
                     const double detectable_angle_rad, const double random_noise_standard_deviation_rad,
                     const double bias_noise_standard_deviation_rad, const double intensity_lower_threshold_percent,
                     const SolarRadiationPressureEnvironment* srp_environment, const LocalCelestialInformation* local_celestial_information)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      intensity_lower_threshold_percent_(intensity_lower_threshold_percent),
      detectable_angle_rad_(detectable_angle_rad),
      srp_environment_(srp_environment),
      local_celestial_information_(local_celestial_information) {
  Initialize(random_noise_standard_deviation_rad, bias_noise_standard_deviation_rad);
}

SunSensor::SunSensor(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                     const math::Quaternion& quaternion_b2c, const double detectable_angle_rad, const double random_noise_standard_deviation_rad,
                     const double bias_noise_standard_deviation_rad, const double intensity_lower_threshold_percent,
                     const SolarRadiationPressureEnvironment* srp_environment, const LocalCelestialInformation* local_celestial_information)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      intensity_lower_threshold_percent_(intensity_lower_threshold_percent),
      detectable_angle_rad_(detectable_angle_rad),
      srp_environment_(srp_environment),
      local_celestial_information_(local_celestial_information) {
  Initialize(random_noise_standard_deviation_rad, bias_noise_standard_deviation_rad);
}

void SunSensor::Initialize(const double random_noise_standard_deviation_rad, const double bias_noise_standard_deviation_rad) {
  // Bias
  NormalRand nr(0.0, bias_noise_standard_deviation_rad, global_randomization.MakeSeed());
  bias_noise_alpha_rad_ += nr;
  bias_noise_beta_rad_ += nr;

  // Normal Random
  random_noise_alpha_.SetParameters(0.0, random_noise_standard_deviation_rad);  // global_randomization.MakeSeed()
  random_noise_beta_.SetParameters(0.0, random_noise_standard_deviation_rad);   // global_randomization.MakeSeed()
}
void SunSensor::MainRoutine(const int time_count) {
  UNUSED(time_count);

  Measure();
}

void SunSensor::Measure() {
  math::Vector<3> sun_pos_b = local_celestial_information_->GetPositionFromSpacecraft_b_m("SUN");
  math::Vector<3> sun_dir_b = sun_pos_b.CalcNormalizedVector();

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

    measured_sun_direction_c_ = measured_sun_direction_c_.CalcNormalizedVector();
  } else {
    measured_sun_direction_c_ = math::Vector<3>(0);
    alpha_rad_ = 0.0;
    beta_rad_ = 0.0;
  }

  CalcSolarIlluminance();
}

void SunSensor::SunDetectionJudgement() {
  math::Vector<3> sun_direction_c = sun_direction_true_c_.CalcNormalizedVector();

  double sun_angle_ = acos(sun_direction_c[2]);

  if (solar_illuminance_W_m2_ < intensity_lower_threshold_percent_ / 100.0 * srp_environment_->GetSolarConstant_W_m2()) {
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
  math::Vector<3> sun_direction_c = sun_direction_true_c_.CalcNormalizedVector();
  double sun_angle_ = acos(sun_direction_c[2]);

  if (sun_angle_ > math::pi_2) {
    solar_illuminance_W_m2_ = 0.0;
    return;
  }

  double power_density = srp_environment_->GetPowerDensity_W_m2();
  solar_illuminance_W_m2_ = power_density * cos(sun_angle_);
  // TODO: Take into account the effects of albedo.
}

double SunSensor::TanRange(double x) {
  if (x > math::pi_2) x = math::pi - x;
  if (x < -math::pi_2) x = -math::pi - x;
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

SunSensor InitSunSensor(ClockGenerator* clock_generator, int ss_id, std::string file_name, const SolarRadiationPressureEnvironment* srp_environment,
                        const LocalCelestialInformation* local_celestial_information) {
  IniAccess ss_conf(file_name);
  const char* sensor_name = "SUN_SENSOR_";
  const std::string section_tmp = sensor_name + std::to_string(static_cast<long long>(ss_id));
  const char* Section = section_tmp.c_str();

  int prescaler = ss_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  math::Quaternion quaternion_b2c;
  ss_conf.ReadQuaternion(Section, "quaternion_b2c", quaternion_b2c);

  double detectable_angle_deg = 0.0, detectable_angle_rad = 0.0;
  detectable_angle_deg = ss_conf.ReadDouble(Section, "field_of_view_deg");
  detectable_angle_rad = math::pi / 180.0 * detectable_angle_deg;

  double nr_stddev = 0.0;
  nr_stddev = ss_conf.ReadDouble(Section, "white_noise_standard_deviation_deg");
  nr_stddev *= math::pi / 180.0;

  double nr_bias_stddev = 0.0;
  nr_bias_stddev = ss_conf.ReadDouble(Section, "bias_standard_deviation_deg");
  nr_bias_stddev *= math::pi / 180.0;

  double intensity_lower_threshold_percent;
  intensity_lower_threshold_percent = ss_conf.ReadDouble(Section, "intensity_lower_threshold_percent");

  SunSensor ss(prescaler, clock_generator, ss_id, quaternion_b2c, detectable_angle_rad, nr_stddev, nr_bias_stddev, intensity_lower_threshold_percent,
               srp_environment, local_celestial_information);
  return ss;
}

SunSensor InitSunSensor(ClockGenerator* clock_generator, PowerPort* power_port, int ss_id, std::string file_name,
                        const SolarRadiationPressureEnvironment* srp_environment, const LocalCelestialInformation* local_celestial_information) {
  IniAccess ss_conf(file_name);
  const char* sensor_name = "SUN_SENSOR_";
  const std::string section_tmp = sensor_name + std::to_string(static_cast<long long>(ss_id));
  const char* Section = section_tmp.c_str();

  int prescaler = ss_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  math::Quaternion quaternion_b2c;
  ss_conf.ReadQuaternion(Section, "quaternion_b2c", quaternion_b2c);

  double detectable_angle_deg = 0.0, detectable_angle_rad = 0.0;
  detectable_angle_deg = ss_conf.ReadDouble(Section, "field_of_view_deg");
  detectable_angle_rad = math::pi / 180.0 * detectable_angle_deg;

  double nr_stddev = 0.0;
  nr_stddev = ss_conf.ReadDouble(Section, "white_noise_standard_deviation_deg");
  nr_stddev *= math::pi / 180.0;

  double nr_bias_stddev = 0.0;
  nr_bias_stddev = ss_conf.ReadDouble(Section, "bias_standard_deviation_deg");
  nr_bias_stddev *= math::pi / 180.0;

  double intensity_lower_threshold_percent;
  intensity_lower_threshold_percent = ss_conf.ReadDouble(Section, "intensity_lower_threshold_percent");

  power_port->InitializeWithInitializeFile(file_name);

  SunSensor ss(prescaler, clock_generator, power_port, ss_id, quaternion_b2c, detectable_angle_rad, nr_stddev, nr_bias_stddev,
               intensity_lower_threshold_percent, srp_environment, local_celestial_information);
  return ss;
}
