/**
 * @file InitGyro.cpp
 * @brief Initialize functions for gyro sensor
 */
#include "InitGyro.hpp"

#include <Interface/InitInput/IniAccess.h>

#include "../Abstract/InitializeSensorBase.hpp"

Gyro InitGyro(ClockGenerator* clock_gen, int sensor_id, const std::string fname, double compo_step_time, const Dynamics* dynamics) {
  IniAccess gyro_conf(fname);
  char GSection[30] = "GYRO";

  Quaternion q_b2c;
  gyro_conf.ReadQuaternion(GSection, "quaternion_b2c", q_b2c);
  int prescaler = gyro_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // SensorBase
  SensorBase<3> sensor_base = ReadSensorBaseInformation<3>(fname, compo_step_time * (double)(prescaler), "Gyro", "rad_s");

  Gyro gyro(prescaler, clock_gen, sensor_base, sensor_id, q_b2c, dynamics);

  return gyro;
}

Gyro InitGyro(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
              const Dynamics* dynamics) {
  IniAccess gyro_conf(fname);
  char GSection[30] = "GYRO";

  Quaternion q_b2c;
  gyro_conf.ReadQuaternion(GSection, "quaternion_b2c", q_b2c);
  int prescaler = gyro_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // SensorBase
  SensorBase<3> sensor_base = ReadSensorBaseInformation<3>(fname, compo_step_time * (double)(prescaler), "Gyro", "rad_s");

  // PowerPort
  double minimum_voltage = gyro_conf.ReadDouble(GSection, "minimum_voltage_V");
  power_port->SetMinimumVoltage(minimum_voltage);
  double assumed_power_consumption = gyro_conf.ReadDouble(GSection, "assumed_power_consumption_W");
  power_port->SetAssumedPowerConsumption(assumed_power_consumption);

  Gyro gyro(prescaler, clock_gen, power_port, sensor_base, sensor_id, q_b2c, dynamics);
  return gyro;
}
