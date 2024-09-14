/**
 * @file gyro_sensor.cpp
 * @brief Class to emulate gyro sensor (angular velocity sensor)
 */

#include "gyro_sensor.hpp"

#include <setting_file_reader/initialize_file_access.hpp>

GyroSensor::GyroSensor(const int prescaler, ClockGenerator* clock_generator, Sensor& sensor_base, const unsigned int sensor_id,
                       const s2e::math::Quaternion& quaternion_b2c, const Dynamics* dynamics)
    : Component(prescaler, clock_generator), Sensor(sensor_base), sensor_id_(sensor_id), quaternion_b2c_(quaternion_b2c), dynamics_(dynamics) {}

GyroSensor::GyroSensor(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, Sensor& sensor_base, const unsigned int sensor_id,
                       const s2e::math::Quaternion& quaternion_b2c, const Dynamics* dynamics)
    : Component(prescaler, clock_generator, power_port),
      Sensor(sensor_base),
      sensor_id_(sensor_id),
      quaternion_b2c_(quaternion_b2c),
      dynamics_(dynamics) {}

GyroSensor::~GyroSensor() {}

void GyroSensor::MainRoutine(const int time_count) {
  UNUSED(time_count);

  angular_velocity_c_rad_s_ = quaternion_b2c_.FrameConversion(dynamics_->GetAttitude().GetAngularVelocity_b_rad_s());  // Convert frame
  angular_velocity_c_rad_s_ = Measure(angular_velocity_c_rad_s_);                                                      // Add noises
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

GyroSensor InitGyroSensor(ClockGenerator* clock_generator, int sensor_id, const std::string file_name, double component_step_time_s,
                          const Dynamics* dynamics) {
  IniAccess gyro_conf(file_name);
  const char* sensor_name = "GYRO_SENSOR_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* GSection = section_name.c_str();

  s2e::math::Quaternion quaternion_b2c;
  gyro_conf.ReadQuaternion(GSection, "quaternion_b2c", quaternion_b2c);
  int prescaler = gyro_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // Sensor
  Sensor<kGyroDimension> sensor_base =
      ReadSensorInformation<kGyroDimension>(file_name, component_step_time_s * (double)(prescaler), GSection, "rad_s");

  GyroSensor gyro(prescaler, clock_generator, sensor_base, sensor_id, quaternion_b2c, dynamics);

  return gyro;
}

GyroSensor InitGyroSensor(ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const std::string file_name,
                          double component_step_time_s, const Dynamics* dynamics) {
  IniAccess gyro_conf(file_name);
  const char* sensor_name = "GYRO_SENSOR_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* GSection = section_name.c_str();

  s2e::math::Quaternion quaternion_b2c;
  gyro_conf.ReadQuaternion(GSection, "quaternion_b2c", quaternion_b2c);
  int prescaler = gyro_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // Sensor
  Sensor<kGyroDimension> sensor_base =
      ReadSensorInformation<kGyroDimension>(file_name, component_step_time_s * (double)(prescaler), GSection, "rad_s");

  // PowerPort
  power_port->InitializeWithInitializeFile(file_name);

  GyroSensor gyro(prescaler, clock_generator, power_port, sensor_base, sensor_id, quaternion_b2c, dynamics);
  return gyro;
}
