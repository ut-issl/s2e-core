/*
 * @file attitude_observer.cpp
 * @brief Ideal component which can observe attitude
 */

#include "attitude_observer.hpp"

#include <library/initialize/initialize_file_access.hpp>
#include <library/math/constants.hpp>

AttitudeObserver::AttitudeObserver(const int prescaler, ClockGenerator* clock_generator, const double standard_deviation_rad,
                                   const Attitude& attitude)
    : Component(prescaler, clock_generator), angle_noise_(0.0, standard_deviation_rad), attitude_(attitude) {
  direction_noise_.SetParameters(0.0, 1.0);
}

void AttitudeObserver::MainRoutine(const int time_count) {
  UNUSED(time_count);

  // Error calculation
  libra::Vector<3> random_direction;
  random_direction[0] = direction_noise_;
  random_direction[1] = direction_noise_;
  random_direction[2] = direction_noise_;
  random_direction = random_direction.CalcNormalizedVector();

  double error_angle_rad = angle_noise_;
  libra::Quaternion error_quaternion(random_direction, error_angle_rad);

  observed_quaternion_i2b_ = error_quaternion * attitude_.GetQuaternion_i2b();
}

std::string AttitudeObserver::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "attitude_observer_";
  str_tmp += WriteQuaternion(head + "quaternion", "i2b");

  return str_tmp;
}

std::string AttitudeObserver::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteQuaternion(observed_quaternion_i2b_);

  return str_tmp;
}

AttitudeObserver InitializeAttitudeObserver(ClockGenerator* clock_generator, const std::string file_name, const Attitude& attitude) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("COMPONENT_BASE", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // ForceGenerator
  double error_angle_standard_deviation_deg = ini_file.ReadDouble("ATTITUDE_OBSERVER", "error_angle_standard_deviation_deg");
  double error_angle_standard_deviation_rad = libra::deg_to_rad * error_angle_standard_deviation_deg;
  AttitudeObserver attitude_observer(prescaler, clock_generator, error_angle_standard_deviation_rad, attitude);

  return attitude_observer;
}
