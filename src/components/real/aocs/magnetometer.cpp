/**
 * @file magnetometer.cpp
 * @brief Class to emulate magnetometer
 */
#include "magnetometer.hpp"

#include <library/math/quaternion.hpp>

MagSensor::MagSensor(int prescaler, ClockGenerator* clock_gen, SensorBase& sensor_base, const int sensor_id, const Quaternion& q_b2c,
                     const GeomagneticField* magnet)
    : ComponentBase(prescaler, clock_gen), SensorBase(sensor_base), sensor_id_(sensor_id), q_b2c_(q_b2c), magnet_(magnet) {}
MagSensor::MagSensor(int prescaler, ClockGenerator* clock_gen, PowerPort* power_port, SensorBase& sensor_base, const int sensor_id,
                     const Quaternion& q_b2c, const GeomagneticField* magnet)
    : ComponentBase(prescaler, clock_gen, power_port), SensorBase(sensor_base), sensor_id_(sensor_id), q_b2c_(q_b2c), magnet_(magnet) {}
MagSensor::~MagSensor() {}

void MagSensor::MainRoutine(int count) {
  UNUSED(count);

  mag_c_ = q_b2c_.frame_conv(magnet_->GetMagneticField_b_nT());  // Convert frame
  mag_c_ = Measure(mag_c_);                                      // Add noises
}

std::string MagSensor::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string sensor_id = std::to_string(static_cast<long long>(sensor_id_));
  std::string sensor_name = "magnetometer" + sensor_id + "_";
  str_tmp += WriteVector(sensor_name + "measured_magnetic_field", "c", "nT", kMagDim);

  return str_tmp;
}

std::string MagSensor::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(mag_c_);

  return str_tmp;
}
