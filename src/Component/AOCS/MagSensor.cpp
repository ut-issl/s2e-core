#include "MagSensor.h"

#include "../../Library/math/Quaternion.hpp"

MagSensor::MagSensor(
  int prescaler, 
  ClockGenerator* clock_gen,
  SensorBase& sensor_base,
  const int sensor_id,
  const Quaternion& q_b2c,
  const MagEnvironment *magnet)
  : ComponentBase(prescaler, clock_gen), SensorBase(sensor_base), 
  sensor_id_(sensor_id), q_b2c_(q_b2c),
  magnet_(magnet)
{}
MagSensor::MagSensor(
  int prescaler, 
  ClockGenerator* clock_gen,
  PowerPort* power_port,
  SensorBase& sensor_base,
  const int sensor_id,
  const Quaternion& q_b2c,
  const MagEnvironment *magnet)
  : ComponentBase(prescaler, clock_gen, power_port), SensorBase(sensor_base), 
  sensor_id_(sensor_id), q_b2c_(q_b2c),
  magnet_(magnet)
{}
MagSensor::~MagSensor()
{}

void MagSensor::MainRoutine(int count)
{
  mag_c_ = q_b2c_.frame_conv(magnet_->GetMag_b()); //Convert frame
  mag_c_ = Measure(mag_c_); //Add noises
}

std::string MagSensor::GetLogHeader() const
{
  std::string str_tmp = "";
  const std::string st_sensor_id = std::to_string(static_cast<long long>(sensor_id_));
  const char *cs = st_sensor_id.data();
  std::string MSSection = "mag_sensor";
  str_tmp += WriteVector(MSSection+cs, "c", "nT", kMagDim);

  return str_tmp;
}

std::string MagSensor::GetLogValue() const
{
  std::string str_tmp = "";

  str_tmp += WriteVector(mag_c_);

  return str_tmp;
}
