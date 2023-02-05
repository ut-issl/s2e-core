/*
 * @file STT.cpp
 * @brief Class to emulate star tracker
 */

#include "STT.h"

#include <Interface/LogOutput/LogUtility.h>
#include <Library/math/GlobalRand.h>

#include <Environment/Global/PhysicalConstants.hpp>
#include <Library/math/Constant.hpp>
#include <Library/math/Matrix.hpp>
#include <string>

using namespace std;
using namespace libra;

STT::STT(const int prescaler, ClockGenerator* clock_gen, const int id, const libra::Quaternion& q_b2c, const double sigma_ortho,
         const double sigma_sight, const double step_time, const unsigned int output_delay, const unsigned int output_interval,
         const double sun_forbidden_angle, const double earth_forbidden_angle, const double moon_forbidden_angle, const double capture_rate,
         const Dynamics* dynamics, const LocalEnvironment* local_env)
    : ComponentBase(prescaler, clock_gen),
      id_(id),
      q_b2c_(q_b2c),
      rot_(g_rand.MakeSeed()),
      n_ortho_(0.0, sigma_ortho, g_rand.MakeSeed()),
      n_sight_(0.0, sigma_sight, g_rand.MakeSeed()),
      pos_(0),
      step_time_(step_time),
      output_delay_(output_delay),
      output_interval_(output_interval),
      count_(0),
      sun_forbidden_angle_(sun_forbidden_angle),
      earth_forbidden_angle_(earth_forbidden_angle),
      moon_forbidden_angle_(moon_forbidden_angle),
      capture_rate_(capture_rate),
      dynamics_(dynamics),
      local_env_(local_env) {
  Initialize();
}
STT::STT(const int prescaler, ClockGenerator* clock_gen, PowerPort* power_port, const int id, const libra::Quaternion& q_b2c,
         const double sigma_ortho, const double sigma_sight, const double step_time, const unsigned int output_delay,
         const unsigned int output_interval, const double sun_forbidden_angle, const double earth_forbidden_angle, const double moon_forbidden_angle,
         const double capture_rate, const Dynamics* dynamics, const LocalEnvironment* local_env)
    : ComponentBase(prescaler, clock_gen, power_port),
      id_(id),
      q_b2c_(q_b2c),
      rot_(g_rand.MakeSeed()),
      n_ortho_(0.0, sigma_ortho, g_rand.MakeSeed()),
      n_sight_(0.0, sigma_sight, g_rand.MakeSeed()),
      pos_(0),
      step_time_(step_time),
      output_delay_(output_delay),
      output_interval_(output_interval),
      count_(0),
      sun_forbidden_angle_(sun_forbidden_angle),
      earth_forbidden_angle_(earth_forbidden_angle),
      moon_forbidden_angle_(moon_forbidden_angle),
      capture_rate_(capture_rate),
      dynamics_(dynamics),
      local_env_(local_env) {
  Initialize();
}

void STT::Initialize() {
  q_stt_i2c_ = Quaternion(0.0, 0.0, 0.0, 1.0);

  // Decide delay buffer size
  MAX_DELAY = int(output_delay_ * 2 / step_time_);
  if (MAX_DELAY <= 0) MAX_DELAY = 1;
  vector<Quaternion> temp(MAX_DELAY);
  q_buffer_ = temp;
  // Initialize delay buffer
  for (int i = 0; i < MAX_DELAY; ++i) {
    q_buffer_[i] = q_stt_i2c_;
  }

  sight_ = Vector<3>(0.0);
  ortho1_ = Vector<3>(0.0);
  ortho2_ = Vector<3>(0.0);
  sight_[0] = 1.0;   //(1,0,0)@Component coordinates, viewing direction
  ortho1_[1] = 1.0;  //(0,1,0)@Component coordinates, line-of-sight orthogonal direction
  ortho2_[2] = 1.0;  //(0,0,1)@Component coordinates, line-of-sight orthogonal direction

  error_flag_ = true;
}
Quaternion STT::measure(const LocalCelestialInformation* local_celes_info, const Attitude* attinfo) {
  update(local_celes_info, attinfo);  // update delay buffer
  if (count_ == 0) {
    int hist = pos_ - output_delay_ - 1;
    if (hist < 0) {
      hist += MAX_DELAY;
    }
    q_stt_i2c_ = q_buffer_[hist];
  }
  if (++count_ == output_interval_) {
    count_ = 0;
  }

  return q_stt_i2c_;
}

void STT::update(const LocalCelestialInformation* local_celes_info, const Attitude* attinfo) {
  Quaternion q_i2b = attinfo->GetQuaternion_i2b();  // Read true value
  Quaternion q_stt_temp = q_i2b * q_b2c_;           // Convert to component frame
  // Add noise on sight direction
  Quaternion q_sight(sight_, n_sight_);
  // Random noise on orthogonal direction of sight. Range [0:2pi]
  double rot = libra::tau * double(rot_);
  // Calc observation error on orthogonal direction of sight
  Vector<3> rot_axis = cos(rot) * ortho1_ + sin(rot) * ortho2_;
  Quaternion q_ortho(rot_axis, n_ortho_);
  // Judge errors
  AllJudgement(local_celes_info, attinfo);

  // Calc observed quaternion: Inertial frame → STT frame → Rotation around
  // sight →Rotation around orthogonal direction
  q_buffer_[pos_] = q_stt_temp * q_sight * q_ortho;
  // Update delay buffer position
  ++pos_;
  pos_ %= MAX_DELAY;
}

void STT::AllJudgement(const LocalCelestialInformation* local_celes_info, const Attitude* attinfo) {
  int judgement = 0;
  judgement = SunJudgement(local_celes_info->GetPosFromSC_b("SUN"));
  judgement += EarthJudgement(local_celes_info->GetPosFromSC_b("EARTH"));
  judgement += MoonJudgement(local_celes_info->GetPosFromSC_b("MOON"));
  judgement += CaptureRateJudgement(attinfo->GetOmega_b());
  if (judgement > 0)
    error_flag_ = true;
  else
    error_flag_ = false;
}

int STT::SunJudgement(const libra::Vector<3>& sun_b) {
  Quaternion q_c2b = q_b2c_.conjugate();
  Vector<3> sight_b = q_c2b.frame_conv(sight_);
  double sun_angle_rad = CalAngleVect_rad(sun_b, sight_b);
  if (sun_angle_rad < sun_forbidden_angle_)
    return 1;
  else
    return 0;
}

int STT::EarthJudgement(const libra::Vector<3>& earth_b) {
  Quaternion q_c2b = q_b2c_.conjugate();
  Vector<3> sight_b = q_c2b.frame_conv(sight_);
  double earth_size_rad = atan2(environment::earth_equatorial_radius_m,
                                norm(earth_b));                           // angles between sat<->earth_center & sat<->earth_edge
  double earth_center_angle_rad = CalAngleVect_rad(earth_b, sight_b);     // angles between sat<->earth_center & sat_sight
  double earth_edge_angle_rad = earth_center_angle_rad - earth_size_rad;  // angles between sat<->earth_edge & sat_sight
  if (earth_edge_angle_rad < earth_forbidden_angle_)
    return 1;
  else
    return 0;
}

int STT::MoonJudgement(const libra::Vector<3>& moon_b) {
  Quaternion q_c2b = q_b2c_.conjugate();
  Vector<3> sight_b = q_c2b.frame_conv(sight_);
  double moon_angle_rad = CalAngleVect_rad(moon_b, sight_b);
  if (moon_angle_rad < moon_forbidden_angle_)
    return 1;
  else
    return 0;
}

int STT::CaptureRateJudgement(const libra::Vector<3>& omega_b) {
  double omega_norm = norm(omega_b);
  if (omega_norm > capture_rate_)
    return 1;
  else
    return 0;
}

std::string STT::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string sensor_id = std::to_string(static_cast<long long>(id_));
  std::string sensor_name = "stt" + sensor_id + "_";

  str_tmp += WriteQuaternion(sensor_name + "measured_quaternion", "i2c");
  str_tmp += WriteScalar(sensor_name + "error_flag");

  return str_tmp;
}

std::string STT::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteQuaternion(q_stt_i2c_);
  str_tmp += WriteScalar(double(error_flag_));

  return str_tmp;
}

double STT::CalAngleVect_rad(const Vector<3>& vect1, const Vector<3>& vect2) {
  Vector<3> vect1_normal(vect1);
  normalize(vect1_normal);  // Normalize Vector1
  Vector<3> vect2_normal(vect2);
  normalize(vect2_normal);                                      // Normalize Vector2
  double cosTheta = inner_product(vect1_normal, vect2_normal);  // Calc cos value
  double theta_rad = acos(cosTheta);
  return theta_rad;
}

void STT::MainRoutine(int count) {
  UNUSED(count);

  measure(&(local_env_->GetCelesInfo()), &(dynamics_->GetAttitude()));
}
