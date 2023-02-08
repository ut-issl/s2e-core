/**
 * @file InitMagSensor.cpp
 * @brief Initialize functions for magnetometer
 */
#include "InitMagSensor.hpp"

#include "../Abstract/InitializeSensorBase.hpp"
#include "interface/initialize/initialize_file_access.hpp"

MagSensor InitMagSensor(ClockGenerator* clock_gen, int sensor_id, const std::string fname, double compo_step_time, const MagEnvironment* magnet) {
  IniAccess magsensor_conf(fname);
  const char* sensor_name = "MAGNETOMETER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* MSSection = section_name.c_str();

  int prescaler = magsensor_conf.ReadInt(MSSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Quaternion q_b2c;
  magsensor_conf.ReadQuaternion(MSSection, "quaternion_b2c", q_b2c);

  // SensorBase
  SensorBase<kMagDim> sensor_base = ReadSensorBaseInformation<kMagDim>(fname, compo_step_time * (double)(prescaler), MSSection, "nT");

  MagSensor magsensor(prescaler, clock_gen, sensor_base, sensor_id, q_b2c, magnet);
  return magsensor;
}

MagSensor InitMagSensor(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
                        const MagEnvironment* magnet) {
  IniAccess magsensor_conf(fname);
  const char* sensor_name = "MAGNETOMETER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* MSSection = section_name.c_str();

  int prescaler = magsensor_conf.ReadInt(MSSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Quaternion q_b2c;
  magsensor_conf.ReadQuaternion(MSSection, "quaternion_b2c", q_b2c);

  // SensorBase
  SensorBase<kMagDim> sensor_base = ReadSensorBaseInformation<kMagDim>(fname, compo_step_time * (double)(prescaler), MSSection, "nT");

  // PowerPort
  power_port->InitializeWithInitializeFile(fname);

  MagSensor magsensor(prescaler, clock_gen, power_port, sensor_base, sensor_id, q_b2c, magnet);
  return magsensor;
}
