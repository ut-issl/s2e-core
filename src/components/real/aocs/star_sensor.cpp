/*
 * @file star_sensor.cpp
 * @brief Class to emulate star tracker
 */

#include "star_sensor.hpp"

#include <environment/global/physical_constants.hpp>
#include <library/logger/log_utility.hpp>
#include <library/math/constants.hpp>
#include <library/math/matrix.hpp>
#include <library/randomization/global_randomization.hpp>
#include <string>

using namespace std;
using namespace libra;

STT::STT(const int prescaler, ClockGenerator* clock_generator, const int component_id, const libra::Quaternion& quaternion_b2c,
         const double sigma_ortho, const double sigma_sight, const double step_time, const unsigned int output_delay,
         const unsigned int output_interval, const double sun_forbidden_angle, const double earth_forbidden_angle, const double moon_forbidden_angle,
         const double capture_rate, const Dynamics* dynamics, const LocalEnvironment* local_env)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      rotation_noise_(global_randomization.MakeSeed()),
      orthogonal_direction_noise_(0.0, sigma_ortho, global_randomization.MakeSeed()),
      sight_direction_noise_(0.0, sigma_sight, global_randomization.MakeSeed()),
      buffer_position_(0),
      step_time_s_(step_time),
      output_delay_(output_delay),
      output_interval_(output_interval),
      update_count_(0),
      sun_forbidden_angle_rad_(sun_forbidden_angle),
      earth_forbidden_angle_rad_(earth_forbidden_angle),
      moon_forbidden_angle_rad_(moon_forbidden_angle),
      capture_rate_limit_rad_s_(capture_rate),
      dynamics_(dynamics),
      local_environment_(local_env) {
  Initialize();
}
STT::STT(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id, const libra::Quaternion& quaternion_b2c,
         const double sigma_ortho, const double sigma_sight, const double step_time, const unsigned int output_delay,
         const unsigned int output_interval, const double sun_forbidden_angle, const double earth_forbidden_angle, const double moon_forbidden_angle,
         const double capture_rate, const Dynamics* dynamics, const LocalEnvironment* local_env)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      rotation_noise_(global_randomization.MakeSeed()),
      orthogonal_direction_noise_(0.0, sigma_ortho, global_randomization.MakeSeed()),
      sight_direction_noise_(0.0, sigma_sight, global_randomization.MakeSeed()),
      buffer_position_(0),
      step_time_s_(step_time),
      output_delay_(output_delay),
      output_interval_(output_interval),
      update_count_(0),
      sun_forbidden_angle_rad_(sun_forbidden_angle),
      earth_forbidden_angle_rad_(earth_forbidden_angle),
      moon_forbidden_angle_rad_(moon_forbidden_angle),
      capture_rate_limit_rad_s_(capture_rate),
      dynamics_(dynamics),
      local_environment_(local_env) {
  Initialize();
}

void STT::Initialize() {
  measured_quaternion_i2c_ = Quaternion(0.0, 0.0, 0.0, 1.0);

  // Decide delay buffer size
  max_delay_ = int(output_delay_ * 2 / step_time_s_);
  if (max_delay_ <= 0) max_delay_ = 1;
  vector<Quaternion> temp(max_delay_);
  delay_buffer_ = temp;
  // Initialize delay buffer
  for (int i = 0; i < max_delay_; ++i) {
    delay_buffer_[i] = measured_quaternion_i2c_;
  }

  sight_direction_c_ = Vector<3>(0.0);
  first_orthogonal_direction_c = Vector<3>(0.0);
  second_orthogonal_direction_c = Vector<3>(0.0);
  sight_direction_c_[0] = 1.0;             //(1,0,0)@Component coordinates, viewing direction
  first_orthogonal_direction_c[1] = 1.0;   //(0,1,0)@Component coordinates, line-of-sight orthogonal direction
  second_orthogonal_direction_c[2] = 1.0;  //(0,0,1)@Component coordinates, line-of-sight orthogonal direction

  error_flag_ = true;
}
Quaternion STT::measure(const LocalCelestialInformation* local_celestial_information, const Attitude* attitude) {
  update(local_celestial_information, attitude);  // update delay buffer
  if (update_count_ == 0) {
    int hist = buffer_position_ - output_delay_ - 1;
    if (hist < 0) {
      hist += max_delay_;
    }
    measured_quaternion_i2c_ = delay_buffer_[hist];
  }
  if (++update_count_ == output_interval_) {
    update_count_ = 0;
  }

  return measured_quaternion_i2c_;
}

void STT::update(const LocalCelestialInformation* local_celestial_information, const Attitude* attitude) {
  Quaternion quaternion_i2b = attitude->GetQuaternion_i2b();  // Read true value
  Quaternion q_stt_temp = quaternion_i2b * quaternion_b2c_;   // Convert to component frame
  // Add noise on sight direction
  Quaternion q_sight(sight_direction_c_, sight_direction_noise_);
  // Random noise on orthogonal direction of sight. Range [0:2pi]
  double rot = libra::tau * double(rotation_noise_);
  // Calc observation error on orthogonal direction of sight
  Vector<3> rot_axis = cos(rot) * first_orthogonal_direction_c + sin(rot) * second_orthogonal_direction_c;
  Quaternion q_ortho(rot_axis, orthogonal_direction_noise_);
  // Judge errors
  AllJudgement(local_celestial_information, attitude);

  // Calc observed quaternion: Inertial frame → STT frame → Rotation around
  // sight →Rotation around orthogonal direction
  delay_buffer_[buffer_position_] = q_stt_temp * q_sight * q_ortho;
  // Update delay buffer position
  ++buffer_position_;
  buffer_position_ %= max_delay_;
}

void STT::AllJudgement(const LocalCelestialInformation* local_celestial_information, const Attitude* attitude) {
  int judgement = 0;
  judgement = SunJudgement(local_celestial_information->GetPositionFromSpacecraft_b_m("SUN"));
  judgement += EarthJudgement(local_celestial_information->GetPositionFromSpacecraft_b_m("EARTH"));
  judgement += MoonJudgement(local_celestial_information->GetPositionFromSpacecraft_b_m("MOON"));
  judgement += CaptureRateJudgement(attitude->GetOmega_b());
  if (judgement > 0)
    error_flag_ = true;
  else
    error_flag_ = false;
}

int STT::SunJudgement(const libra::Vector<3>& sun_b) {
  Quaternion q_c2b = quaternion_b2c_.Conjugate();
  Vector<3> sight_b = q_c2b.FrameConversion(sight_direction_c_);
  double sun_angle_rad = CalAngleVector_rad(sun_b, sight_b);
  if (sun_angle_rad < sun_forbidden_angle_rad_)
    return 1;
  else
    return 0;
}

int STT::EarthJudgement(const libra::Vector<3>& earth_b) {
  Quaternion q_c2b = quaternion_b2c_.Conjugate();
  Vector<3> sight_b = q_c2b.FrameConversion(sight_direction_c_);
  double earth_size_rad = atan2(environment::earth_equatorial_radius_m,
                                CalcNorm(earth_b));                       // angles between sat<->earth_center & sat<->earth_edge
  double earth_center_angle_rad = CalAngleVector_rad(earth_b, sight_b);   // angles between sat<->earth_center & sat_sight
  double earth_edge_angle_rad = earth_center_angle_rad - earth_size_rad;  // angles between sat<->earth_edge & sat_sight
  if (earth_edge_angle_rad < earth_forbidden_angle_rad_)
    return 1;
  else
    return 0;
}

int STT::MoonJudgement(const libra::Vector<3>& moon_b) {
  Quaternion q_c2b = quaternion_b2c_.Conjugate();
  Vector<3> sight_b = q_c2b.FrameConversion(sight_direction_c_);
  double moon_angle_rad = CalAngleVector_rad(moon_b, sight_b);
  if (moon_angle_rad < moon_forbidden_angle_rad_)
    return 1;
  else
    return 0;
}

int STT::CaptureRateJudgement(const libra::Vector<3>& omega_b_rad_s) {
  double omega_norm = CalcNorm(omega_b_rad_s);
  if (omega_norm > capture_rate_limit_rad_s_)
    return 1;
  else
    return 0;
}

std::string STT::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string sensor_id = std::to_string(static_cast<long long>(component_id_));
  std::string sensor_name = "stt" + sensor_id + "_";

  str_tmp += WriteQuaternion(sensor_name + "measured_quaternion", "i2c");
  str_tmp += WriteScalar(sensor_name + "error_flag");

  return str_tmp;
}

std::string STT::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteQuaternion(measured_quaternion_i2c_);
  str_tmp += WriteScalar(double(error_flag_));

  return str_tmp;
}

double STT::CalAngleVector_rad(const Vector<3>& vector1, const Vector<3>& vector2) {
  Vector<3> vect1_normal(vector1);
  Normalize(vect1_normal);  // Normalize Vector1
  Vector<3> vect2_normal(vector2);
  Normalize(vect2_normal);                                     // Normalize Vector2
  double cosTheta = InnerProduct(vect1_normal, vect2_normal);  // Calc cos value
  double theta_rad = acos(cosTheta);
  return theta_rad;
}

void STT::MainRoutine(int count) {
  UNUSED(count);

  measure(&(local_environment_->GetCelestialInformation()), &(dynamics_->GetAttitude()));
}
