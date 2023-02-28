/**
 * @file magnetometer.cpp
 * @brief Class to emulate magnetometer
 */
#include "magnetometer.hpp"

#include <library/math/quaternion.hpp>

Magnetometer::Magnetometer(int prescaler, ClockGenerator* clock_generator, Sensor& sensor_base, const unsigned int sensor_id,
                           const Quaternion& quaternion_b2c, const GeomagneticField* geomagnetic_field)
    : Component(prescaler, clock_generator),
      Sensor(sensor_base),
      sensor_id_(sensor_id),
      quaternion_b2c_(quaternion_b2c),
      geomagnetic_field_(geomagnetic_field) {}
Magnetometer::Magnetometer(int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, Sensor& sensor_base, const unsigned int sensor_id,
                           const Quaternion& quaternion_b2c, const GeomagneticField* geomagnetic_field)
    : Component(prescaler, clock_generator, power_port),
      Sensor(sensor_base),
      sensor_id_(sensor_id),
      quaternion_b2c_(quaternion_b2c),
      geomagnetic_field_(geomagnetic_field) {}
Magnetometer::~Magnetometer() {}

void Magnetometer::MainRoutine(const int time_count) {
  UNUSED(time_count);

  magnetic_field_c_nT_ = quaternion_b2c_.FrameConversion(geomagnetic_field_->GetGeomagneticField_b_nT());  // Convert frame
  magnetic_field_c_nT_ = Measure(magnetic_field_c_nT_);                                                    // Add noises
}

std::string Magnetometer::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string sensor_id = std::to_string(static_cast<long long>(sensor_id_));
  std::string sensor_name = "magnetometer" + sensor_id + "_";
  str_tmp += WriteVector(sensor_name + "measured_magnetic_field", "c", "nT", kMagDim);

  return str_tmp;
}

std::string Magnetometer::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(magnetic_field_c_nT_);

  return str_tmp;
}
