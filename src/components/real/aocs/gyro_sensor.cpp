/**
 * @file gyro_sensor.cpp
 * @brief Class to emulate gyro sensor (angular velocity sensor)
 */

#include "gyro_sensor.hpp"

GyroSensor::GyroSensor(const int prescaler, ClockGenerator* clock_generator, Sensor& sensor_base, const int sensor_id,
                       const Quaternion& quaternion_b2c, const Dynamics* dynamics)
    : Component(prescaler, clock_generator), Sensor(sensor_base), sensor_id_(sensor_id), quaternion_b2c_(quaternion_b2c), dynamics_(dynamics) {}

GyroSensor::GyroSensor(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, Sensor& sensor_base, const int sensor_id,
                       const libra::Quaternion& quaternion_b2c, const Dynamics* dynamics)
    : Component(prescaler, clock_generator, power_port),
      Sensor(sensor_base),
      sensor_id_(sensor_id),
      quaternion_b2c_(quaternion_b2c),
      dynamics_(dynamics) {}

GyroSensor::~GyroSensor() {}

void GyroSensor::MainRoutine(const int time_count) {
  UNUSED(time_count);

  angular_velocity_c_rad_s_ = quaternion_b2c_.FrameConversion(dynamics_->GetAttitude().GetOmega_b());  // Convert frame
  angular_velocity_c_rad_s_ = Measure(angular_velocity_c_rad_s_);                                      // Add noises
}

std::string GyroSensor::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string sensor_id = std::to_string(static_cast<long long>(sensor_id_));
  std::string sensor_name = "gyro_sensor" + sensor_id + "_";
  str_tmp += WriteVector(sensor_name + "measured_angular_velocity", "c", "rad/s", kGyroDimension);

  return str_tmp;
}

std::string GyroSensor::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(angular_velocity_c_rad_s_);

  return str_tmp;
}
