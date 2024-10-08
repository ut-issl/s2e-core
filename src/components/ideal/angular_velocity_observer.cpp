/*
 * @file angular_velocity_observer.cpp
 * @brief Ideal component which can observe angular velocity
 */

#include "angular_velocity_observer.hpp"

#include <setting_file_reader/initialize_file_access.hpp>

namespace s2e::components {

AngularVelocityObserver::AngularVelocityObserver(const int prescaler, environment::ClockGenerator* clock_generator, Sensor& sensor_base,
                                                 const dynamics::attitude::Attitude& attitude)
    : Component(prescaler, clock_generator), Sensor(sensor_base), attitude_(attitude) {}

void AngularVelocityObserver::MainRoutine(const int time_count) {
  UNUSED(time_count);
  angular_velocity_b_rad_s_ = Measure(attitude_.GetAngularVelocity_b_rad_s());
}

std::string AngularVelocityObserver::GetLogHeader() const {
  std::string str_tmp = "";

  std::string sensor_name = "angular_velocity_observer_";
  str_tmp += logger::WriteVector(sensor_name + "measured_value", "b", "rad/s", 3);

  return str_tmp;
}

std::string AngularVelocityObserver::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += logger::WriteVector(angular_velocity_b_rad_s_);

  return str_tmp;
}

AngularVelocityObserver InitializeAngularVelocityObserver(environment::ClockGenerator* clock_generator, const std::string file_name,
                                                          double component_step_time_s, const dynamics::attitude::Attitude& attitude) {
  setting_file_reader::IniAccess ini_file(file_name);

  int prescaler = ini_file.ReadInt("COMPONENT_BASE", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // Sensor
  Sensor<3> sensor_base = ReadSensorInformation<3>(file_name, component_step_time_s * (double)(prescaler), "ANGULAR_VELOCITY_OBSERVER", "rad_s");

  AngularVelocityObserver observer(prescaler, clock_generator, sensor_base, attitude);

  return observer;
}

}  // namespace s2e::components
