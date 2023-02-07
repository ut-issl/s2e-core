/**
 * @file InitGyro.cpp
 * @brief Initialize functions for gyro sensor
 */
#include "InitGyro.hpp"

#include <interface/initialize/IniAccess.h>

#include "../Abstract/InitializeSensorBase.hpp"

Gyro InitGyro(ClockGenerator* clock_gen, int sensor_id, const std::string fname, double compo_step_time, const Dynamics* dynamics) {
  IniAccess gyro_conf(fname);
  const char* sensor_name = "GYRO_SENSOR_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* GSection = section_name.c_str();

  Quaternion q_b2c;
  gyro_conf.ReadQuaternion(GSection, "quaternion_b2c", q_b2c);
  int prescaler = gyro_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // SensorBase
  SensorBase<kGyroDim> sensor_base = ReadSensorBaseInformation<kGyroDim>(fname, compo_step_time * (double)(prescaler), GSection, "rad_s");

  Gyro gyro(prescaler, clock_gen, sensor_base, sensor_id, q_b2c, dynamics);

  return gyro;
}

Gyro InitGyro(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
              const Dynamics* dynamics) {
  IniAccess gyro_conf(fname);
  const char* sensor_name = "GYRO_SENSOR_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* GSection = section_name.c_str();

  Quaternion q_b2c;
  gyro_conf.ReadQuaternion(GSection, "quaternion_b2c", q_b2c);
  int prescaler = gyro_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // SensorBase
  SensorBase<kGyroDim> sensor_base = ReadSensorBaseInformation<kGyroDim>(fname, compo_step_time * (double)(prescaler), GSection, "rad_s");

  // PowerPort
  power_port->InitializeWithInitializeFile(fname);

  Gyro gyro(prescaler, clock_gen, power_port, sensor_base, sensor_id, q_b2c, dynamics);
  return gyro;
}
