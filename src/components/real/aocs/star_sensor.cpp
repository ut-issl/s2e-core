/*
 * @file star_sensor.cpp
 * @brief Class to emulate star tracker
 */

#include "star_sensor.hpp"

#include <environment/global/physical_constants.hpp>
#include <logger/log_utility.hpp>
#include <math_physics/math/constants.hpp>
#include <math_physics/math/matrix.hpp>
#include <math_physics/randomization/global_randomization.hpp>
#include <setting_file_reader/initialize_file_access.hpp>
#include <string>

using namespace std;
using namespace s2e::math;

namespace s2e::components {

StarSensor::StarSensor(const int prescaler, environment::ClockGenerator* clock_generator, const int component_id, const s2e::math::Quaternion& quaternion_b2c,
                       const double standard_deviation_orthogonal_direction, const double standard_deviation_sight_direction,
                       const double step_time_s, const unsigned int output_delay, const unsigned int output_interval,
                       const double sun_forbidden_angle_rad, const double earth_forbidden_angle_rad, const double moon_forbidden_angle_rad,
                       const double capture_rate_limit_rad_s, const Dynamics* dynamics, const LocalEnvironment* local_environment)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      rotation_noise_(s2e::randomization::global_randomization.MakeSeed()),
      orthogonal_direction_noise_(0.0, standard_deviation_orthogonal_direction, s2e::randomization::global_randomization.MakeSeed()),
      sight_direction_noise_(0.0, standard_deviation_sight_direction, s2e::randomization::global_randomization.MakeSeed()),
      buffer_position_(0),
      step_time_s_(step_time_s),
      output_delay_(output_delay),
      output_interval_(output_interval),
      update_count_(0),
      sun_forbidden_angle_rad_(sun_forbidden_angle_rad),
      earth_forbidden_angle_rad_(earth_forbidden_angle_rad),
      moon_forbidden_angle_rad_(moon_forbidden_angle_rad),
      capture_rate_limit_rad_s_(capture_rate_limit_rad_s),
      dynamics_(dynamics),
      local_environment_(local_environment) {
  Initialize();
}
StarSensor::StarSensor(const int prescaler, environment::ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                       const s2e::math::Quaternion& quaternion_b2c, const double standard_deviation_orthogonal_direction,
                       const double standard_deviation_sight_direction, const double step_time_s, const unsigned int output_delay,
                       const unsigned int output_interval, const double sun_forbidden_angle_rad, const double earth_forbidden_angle_rad,
                       const double moon_forbidden_angle_rad, const double capture_rate_limit_rad_s, const Dynamics* dynamics,
                       const LocalEnvironment* local_environment)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      rotation_noise_(s2e::randomization::global_randomization.MakeSeed()),
      orthogonal_direction_noise_(0.0, standard_deviation_orthogonal_direction, s2e::randomization::global_randomization.MakeSeed()),
      sight_direction_noise_(0.0, standard_deviation_sight_direction, s2e::randomization::global_randomization.MakeSeed()),
      buffer_position_(0),
      step_time_s_(step_time_s),
      output_delay_(output_delay),
      output_interval_(output_interval),
      update_count_(0),
      sun_forbidden_angle_rad_(sun_forbidden_angle_rad),
      earth_forbidden_angle_rad_(earth_forbidden_angle_rad),
      moon_forbidden_angle_rad_(moon_forbidden_angle_rad),
      capture_rate_limit_rad_s_(capture_rate_limit_rad_s),
      dynamics_(dynamics),
      local_environment_(local_environment) {
  Initialize();
}

void StarSensor::Initialize() {
  measured_quaternion_i2c_ = s2e::math::Quaternion(0.0, 0.0, 0.0, 1.0);

  // Decide delay buffer size
  max_delay_ = int(output_delay_ * 2 / step_time_s_);
  if (max_delay_ <= 0) max_delay_ = 1;
  vector<s2e::math::Quaternion> temp(max_delay_);
  delay_buffer_ = temp;
  // Initialize delay buffer
  for (int i = 0; i < max_delay_; ++i) {
    delay_buffer_[i] = measured_quaternion_i2c_;
  }

  sight_direction_c_ = s2e::math::Vector<3>(0.0);
  first_orthogonal_direction_c = s2e::math::Vector<3>(0.0);
  second_orthogonal_direction_c = s2e::math::Vector<3>(0.0);
  sight_direction_c_[0] = 1.0;             //(1,0,0)@Component coordinates, viewing direction
  first_orthogonal_direction_c[1] = 1.0;   //(0,1,0)@Component coordinates, line-of-sight orthogonal direction
  second_orthogonal_direction_c[2] = 1.0;  //(0,0,1)@Component coordinates, line-of-sight orthogonal direction

  error_flag_ = true;
}
Quaternion StarSensor::Measure(const LocalCelestialInformation* local_celestial_information, const dynamics::attitude::Attitude* attitude) {
  update(local_celestial_information, attitude);  // update delay buffer
  if (update_count_ == 0) {
    int hist = buffer_position_ - output_delay_ - 1;
    if (hist < 0) {
      hist = max_delay_ - 1;
    }
    measured_quaternion_i2c_ = delay_buffer_[hist];
  }
  if (++update_count_ == output_interval_) {
    update_count_ = 0;
  }

  return measured_quaternion_i2c_;
}

void StarSensor::update(const LocalCelestialInformation* local_celestial_information, const dynamics::attitude::Attitude* attitude) {
  Quaternion quaternion_i2b = attitude->GetQuaternion_i2b();  // Read true value
  Quaternion q_stt_temp = quaternion_i2b * quaternion_b2c_;   // Convert to component frame
  // Add noise on sight direction
  Quaternion q_sight(sight_direction_c_, sight_direction_noise_);
  // Random noise on orthogonal direction of sight. Range [0:2pi]
  double rot = s2e::math::tau * double(rotation_noise_);
  // Calc observation error on orthogonal direction of sight
  s2e::math::Vector<3> rot_axis = cos(rot) * first_orthogonal_direction_c + sin(rot) * second_orthogonal_direction_c;
  s2e::math::Quaternion q_ortho(rot_axis, orthogonal_direction_noise_);
  // Judge errors
  AllJudgement(local_celestial_information, attitude);

  // Calc observed quaternion: Inertial frame → StarSensor frame → Rotation around
  // sight →Rotation around orthogonal direction
  delay_buffer_[buffer_position_] = q_stt_temp * q_sight * q_ortho;
  // Update delay buffer position
  ++buffer_position_;
  buffer_position_ %= max_delay_;
}

void StarSensor::AllJudgement(const LocalCelestialInformation* local_celestial_information, const dynamics::attitude::Attitude* attitude) {
  int judgement = 0;
  judgement = SunJudgement(local_celestial_information->GetPositionFromSpacecraft_b_m("SUN"));
  judgement += EarthJudgement(local_celestial_information->GetPositionFromSpacecraft_b_m("EARTH"));
  judgement += MoonJudgement(local_celestial_information->GetPositionFromSpacecraft_b_m("MOON"));
  judgement += CaptureRateJudgement(attitude->GetAngularVelocity_b_rad_s());
  if (judgement > 0)
    error_flag_ = true;
  else
    error_flag_ = false;
}

int StarSensor::SunJudgement(const s2e::math::Vector<3>& sun_b) {
  s2e::math::Quaternion q_c2b = quaternion_b2c_.Conjugate();
  s2e::math::Vector<3> sight_b = q_c2b.FrameConversion(sight_direction_c_);
  double sun_angle_rad = CalAngleVector_rad(sun_b, sight_b);
  if (sun_angle_rad < sun_forbidden_angle_rad_)
    return 1;
  else
    return 0;
}

int StarSensor::EarthJudgement(const s2e::math::Vector<3>& earth_b) {
  s2e::math::Quaternion q_c2b = quaternion_b2c_.Conjugate();
  s2e::math::Vector<3> sight_b = q_c2b.FrameConversion(sight_direction_c_);
  double earth_size_rad = atan2(environment::earth_equatorial_radius_m,
                                earth_b.CalcNorm());                      // angles between sat<->earth_center & sat<->earth_edge
  double earth_center_angle_rad = CalAngleVector_rad(earth_b, sight_b);   // angles between sat<->earth_center & sat_sight
  double earth_edge_angle_rad = earth_center_angle_rad - earth_size_rad;  // angles between sat<->earth_edge & sat_sight
  if (earth_edge_angle_rad < earth_forbidden_angle_rad_)
    return 1;
  else
    return 0;
}

int StarSensor::MoonJudgement(const s2e::math::Vector<3>& moon_b) {
  s2e::math::Quaternion q_c2b = quaternion_b2c_.Conjugate();
  s2e::math::Vector<3> sight_b = q_c2b.FrameConversion(sight_direction_c_);
  double moon_angle_rad = CalAngleVector_rad(moon_b, sight_b);
  if (moon_angle_rad < moon_forbidden_angle_rad_)
    return 1;
  else
    return 0;
}

int StarSensor::CaptureRateJudgement(const s2e::math::Vector<3>& omega_b_rad_s) {
  double omega_norm = omega_b_rad_s.CalcNorm();
  if (omega_norm > capture_rate_limit_rad_s_)
    return 1;
  else
    return 0;
}

std::string StarSensor::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string sensor_id = std::to_string(static_cast<long long>(component_id_));
  std::string sensor_name = "stt" + sensor_id + "_";

  str_tmp += logger::WriteQuaternion(sensor_name + "measured_quaternion", "i2c");
  str_tmp += WriteScalar(sensor_name + "error_flag");

  return str_tmp;
}

std::string StarSensor::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += logger::WriteQuaternion(measured_quaternion_i2c_);
  str_tmp += WriteScalar(double(error_flag_));

  return str_tmp;
}

double StarSensor::CalAngleVector_rad(const Vector<3>& vector1, const Vector<3>& vector2) {
  s2e::math::Vector<3> vect1_normal = vector1.CalcNormalizedVector();
  s2e::math::Vector<3> vect2_normal = vector2.CalcNormalizedVector();

  double cosTheta = InnerProduct(vect1_normal, vect2_normal);  // Calc cos value
  double theta_rad = acos(cosTheta);
  return theta_rad;
}

void StarSensor::MainRoutine(const int time_count) {
  UNUSED(time_count);

  Measure(&(local_environment_->GetCelestialInformation()), &(dynamics_->GetAttitude()));
}

StarSensor InitStarSensor(environment::ClockGenerator* clock_generator, int sensor_id, const string file_name, double component_step_time_s,
                          const Dynamics* dynamics, const LocalEnvironment* local_environment) {
  setting_file_reader::IniAccess STT_conf(file_name);
  const char* sensor_name = "STAR_SENSOR_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* STTSection = section_name.c_str();

  int prescaler = STT_conf.ReadInt(STTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  double step_time_s = component_step_time_s * prescaler;
  s2e::math::Quaternion quaternion_b2c;
  STT_conf.ReadQuaternion(STTSection, "quaternion_b2c", quaternion_b2c);
  double standard_deviation_orthogonal_direction = STT_conf.ReadDouble(STTSection, "standard_deviation_orthogonal_direction_rad");
  double standard_deviation_sight_direction = STT_conf.ReadDouble(STTSection, "standard_deviation_sight_direction_rad");
  double output_delay_sec = STT_conf.ReadDouble(STTSection, "output_delay_s");
  int output_delay = max(int(output_delay_sec / step_time_s), 1);
  double output_interval_sec = STT_conf.ReadDouble(STTSection, "output_interval_s");
  int output_interval = max(int(output_interval_sec / step_time_s), 1);
  double sun_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "sun_exclusion_angle_deg");
  double sun_forbidden_angle_rad = sun_forbidden_angle_deg * s2e::math::pi / 180.0;
  double earth_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "earth_exclusion_angle_deg");
  double earth_forbidden_angle_rad = earth_forbidden_angle_deg * s2e::math::pi / 180.0;
  double moon_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "moon_exclusion_angle_deg");
  double moon_forbidden_angle_rad = moon_forbidden_angle_deg * s2e::math::pi / 180.0;
  double capture_rate_deg_s = STT_conf.ReadDouble(STTSection, "angular_rate_limit_deg_s");
  double capture_rate_rad_s = capture_rate_deg_s * s2e::math::pi / 180.0;

  StarSensor stt(prescaler, clock_generator, sensor_id, quaternion_b2c, standard_deviation_orthogonal_direction, standard_deviation_sight_direction,
                 step_time_s, output_delay, output_interval, sun_forbidden_angle_rad, earth_forbidden_angle_rad, moon_forbidden_angle_rad,
                 capture_rate_rad_s, dynamics, local_environment);
  return stt;
}

StarSensor InitStarSensor(environment::ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const string file_name, double component_step_time_s,
                          const Dynamics* dynamics, const LocalEnvironment* local_environment) {
  setting_file_reader::IniAccess STT_conf(file_name);
  const char* sensor_name = "STAR_SENSOR_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* STTSection = section_name.c_str();

  int prescaler = STT_conf.ReadInt(STTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  double step_time_s = component_step_time_s * prescaler;

  s2e::math::Quaternion quaternion_b2c;
  STT_conf.ReadQuaternion(STTSection, "quaternion_b2c", quaternion_b2c);
  double standard_deviation_orthogonal_direction = STT_conf.ReadDouble(STTSection, "standard_deviation_orthogonal_direction_rad");
  double standard_deviation_sight_direction = STT_conf.ReadDouble(STTSection, "standard_deviation_sight_direction_rad");
  double output_delay_sec = STT_conf.ReadDouble(STTSection, "output_delay_s");
  int output_delay = max(int(output_delay_sec / step_time_s), 1);
  double output_interval_sec = STT_conf.ReadDouble(STTSection, "output_interval_s");
  int output_interval = max(int(output_interval_sec / step_time_s), 1);
  double sun_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "sun_exclusion_angle_deg");
  double sun_forbidden_angle_rad = sun_forbidden_angle_deg * s2e::math::pi / 180.0;
  double earth_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "earth_exclusion_angle_deg");
  double earth_forbidden_angle_rad = earth_forbidden_angle_deg * s2e::math::pi / 180.0;
  double moon_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "moon_exclusion_angle_deg");
  double moon_forbidden_angle_rad = moon_forbidden_angle_deg * s2e::math::pi / 180.0;
  double capture_rate_deg_s = STT_conf.ReadDouble(STTSection, "angular_rate_limit_deg_s");
  double capture_rate_rad_s = capture_rate_deg_s * s2e::math::pi / 180.0;

  power_port->InitializeWithInitializeFile(file_name);

  StarSensor stt(prescaler, clock_generator, power_port, sensor_id, quaternion_b2c, standard_deviation_orthogonal_direction,
                 standard_deviation_sight_direction, step_time_s, output_delay, output_interval, sun_forbidden_angle_rad, earth_forbidden_angle_rad,
                 moon_forbidden_angle_rad, capture_rate_rad_s, dynamics, local_environment);
  return stt;
}

} // namespace s2e::components
