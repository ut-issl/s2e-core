/**
 * @file magnetometer.cpp
 * @brief Class to emulate magnetometer
 */
#include "magnetometer.hpp"

#include <math_physics/math/quaternion.hpp>
#include <setting_file_reader/initialize_file_access.hpp>

Magnetometer::Magnetometer(int prescaler, ClockGenerator* clock_generator, Sensor& sensor_base, const unsigned int sensor_id,
                           const libra::Quaternion& quaternion_b2c, const GeomagneticField* geomagnetic_field)
    : Component(prescaler, clock_generator),
      Sensor(sensor_base),
      sensor_id_(sensor_id),
      quaternion_b2c_(quaternion_b2c),
      geomagnetic_field_(geomagnetic_field) {}
Magnetometer::Magnetometer(int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, Sensor& sensor_base, const unsigned int sensor_id,
                           const libra::Quaternion& quaternion_b2c, const GeomagneticField* geomagnetic_field)
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
  str_tmp += WriteVector(sensor_name + "measured_magnetic_field", "c", "nT", kMagnetometerDimension);

  return str_tmp;
}

std::string Magnetometer::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(magnetic_field_c_nT_);

  return str_tmp;
}

Magnetometer InitMagnetometer(ClockGenerator* clock_generator, int sensor_id, const std::string file_name, double component_step_time_s,
                              const GeomagneticField* geomagnetic_field) {
  IniAccess magsensor_conf(file_name);
  const char* sensor_name = "MAGNETOMETER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* MSSection = section_name.c_str();

  int prescaler = magsensor_conf.ReadInt(MSSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  libra::Quaternion quaternion_b2c;
  magsensor_conf.ReadQuaternion(MSSection, "quaternion_b2c", quaternion_b2c);

  // Sensor
  Sensor<kMagnetometerDimension> sensor_base =
      ReadSensorInformation<kMagnetometerDimension>(file_name, component_step_time_s * (double)(prescaler), MSSection, "nT");

  Magnetometer magsensor(prescaler, clock_generator, sensor_base, sensor_id, quaternion_b2c, geomagnetic_field);
  return magsensor;
}

Magnetometer InitMagnetometer(ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const std::string file_name,
                              double component_step_time_s, const GeomagneticField* geomagnetic_field) {
  IniAccess magsensor_conf(file_name);
  const char* sensor_name = "MAGNETOMETER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(sensor_id));
  const char* MSSection = section_name.c_str();

  int prescaler = magsensor_conf.ReadInt(MSSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  libra::Quaternion quaternion_b2c;
  magsensor_conf.ReadQuaternion(MSSection, "quaternion_b2c", quaternion_b2c);

  // Sensor
  Sensor<kMagnetometerDimension> sensor_base =
      ReadSensorInformation<kMagnetometerDimension>(file_name, component_step_time_s * (double)(prescaler), MSSection, "nT");

  // PowerPort
  power_port->InitializeWithInitializeFile(file_name);

  Magnetometer magsensor(prescaler, clock_generator, power_port, sensor_base, sensor_id, quaternion_b2c, geomagnetic_field);
  return magsensor;
}
