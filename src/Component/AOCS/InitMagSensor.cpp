/**
 * @file InitMagSensor.cpp
 * @brief Initialize functions for magnetometer
 */
#include "InitMagSensor.hpp"

#include "../Abstract/InitializeSensorBase.hpp"
#include "Interface/InitInput/IniAccess.h"

MagSensor InitMagSensor(ClockGenerator* clock_gen, int sensor_id, const std::string fname, double compo_step_time, const MagEnvironment* magnet) {
  IniAccess magsensor_conf(fname);
  char MSSection[30] = "MAGSENSOR";

  int prescaler = magsensor_conf.ReadInt(MSSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Quaternion q_b2c;
  magsensor_conf.ReadQuaternion(MSSection, "quaternion_b2c", q_b2c);

  // SensorBase
  SensorBase<kMagDim> sensor_base = ReadSensorBaseInformation<kMagDim>(fname, compo_step_time * (double)(prescaler), "Magnetometer", "nT");

  MagSensor magsensor(prescaler, clock_gen, sensor_base, sensor_id, q_b2c, magnet);
  return magsensor;
}

MagSensor InitMagSensor(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
                        const MagEnvironment* magnet) {
  IniAccess magsensor_conf(fname);
  char MSSection[30] = "MAGSENSOR";

  int prescaler = magsensor_conf.ReadInt(MSSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Quaternion q_b2c;
  magsensor_conf.ReadQuaternion(MSSection, "quaternion_b2c", q_b2c);

  // SensorBase
  SensorBase<kMagDim> sensor_base = ReadSensorBaseInformation<kMagDim>(fname, compo_step_time * (double)(prescaler), "Magnetometer", "nT");

  // PowerPort
  double minimum_voltage = magsensor_conf.ReadDouble(MSSection, "minimum_voltage_V");
  power_port->SetMinimumVoltage(minimum_voltage);
  double assumed_power_consumption = magsensor_conf.ReadDouble(MSSection, "assumed_power_consumption_W");
  power_port->AddAssumedPowerConsumption(assumed_power_consumption);

  MagSensor magsensor(prescaler, clock_gen, power_port, sensor_base, sensor_id, q_b2c, magnet);
  return magsensor;
}
