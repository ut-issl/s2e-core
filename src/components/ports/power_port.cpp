/**
 * @file power_port.cpp
 * @brief Class to emulate electrical power port
 */

#include "power_port.hpp"

#include <cfloat>
#include <setting_file_reader/initialize_file_access.hpp>
#include <library/utilities/macros.hpp>

PowerPort::PowerPort() : kPortId(-1), current_limit_A_(10.0), minimum_voltage_V_(3.3), assumed_power_consumption_W_(0.0) {
  is_on_ = true;  // power on to work the component
  Initialize();
}

PowerPort::PowerPort(const int port_id, const double current_limit_A)
    : kPortId(port_id), current_limit_A_(current_limit_A), minimum_voltage_V_(3.3), assumed_power_consumption_W_(0.0) {
  Initialize();
}

PowerPort::PowerPort(const int port_id, const double current_limit_A, const double minimum_voltage_V, const double assumed_power_consumption_W)
    : kPortId(port_id),
      current_limit_A_(current_limit_A),
      minimum_voltage_V_(minimum_voltage_V),
      assumed_power_consumption_W_(assumed_power_consumption_W) {
  Initialize();
}

PowerPort::~PowerPort() {}

void PowerPort::Initialize(void) {
  UNUSED(kPortId);  // TODO: consider delete this variable
  voltage_V_ = 0.0f;
  current_consumption_A_ = 0.0f;
}

bool PowerPort::Update(void) {
  // switching
  if (voltage_V_ >= (minimum_voltage_V_ - DBL_EPSILON)) {
    is_on_ = true;
    current_consumption_A_ = assumed_power_consumption_W_ / voltage_V_;
  } else {
    current_consumption_A_ = 0.0;
    is_on_ = false;
  }
  // over current protection
  if (current_consumption_A_ >= (current_limit_A_ - DBL_EPSILON)) {
    current_consumption_A_ = 0.0;
    voltage_V_ = 0.0;
    is_on_ = false;
  }
  return is_on_;
}

bool PowerPort::SetVoltage_V(const double voltage_V) {
  voltage_V_ = voltage_V;
  Update();
  return is_on_;
}

void PowerPort::SubtractAssumedPowerConsumption_W(const double power_W) {
  assumed_power_consumption_W_ -= power_W;
  if (assumed_power_consumption_W_ < 0.0) assumed_power_consumption_W_ = 0.0;
  return;
}

void PowerPort::InitializeWithInitializeFile(const std::string file_name) {
  IniAccess initialize_file(file_name);
  const std::string section_name = "POWER_PORT";

  double minimum_voltage_V = initialize_file.ReadDouble(section_name.c_str(), "minimum_voltage_V");
  this->SetMinimumVoltage_V(minimum_voltage_V);
  double assumed_power_consumption_W = initialize_file.ReadDouble(section_name.c_str(), "assumed_power_consumption_W");
  this->SetAssumedPowerConsumption_W(assumed_power_consumption_W);
}
