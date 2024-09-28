/*
 * @file power_control_unit.cpp
 * @brief Component emulation of Power Control Unit
 */
#include "power_control_unit.hpp"

namespace s2e::components {

PowerControlUnit::PowerControlUnit(environment::ClockGenerator* clock_generator) : Component(1, clock_generator) {}

PowerControlUnit::PowerControlUnit(int prescaler, environment::ClockGenerator* clock_generator) : Component(prescaler, clock_generator) {}

PowerControlUnit::~PowerControlUnit() {}

void PowerControlUnit::MainRoutine(const int time_count) {
  UNUSED(time_count);

  // double current_ = power_ports_[1]->GetCurrentConsumption_A();
}

int PowerControlUnit::ConnectPort(const int port_id, const double current_limit_A) {
  // The port is already used
  if (power_ports_[port_id] != nullptr) return -1;

  power_ports_[port_id] = new PowerPort(port_id, current_limit_A);
  return 0;
}

int PowerControlUnit::ConnectPort(const int port_id, const double current_limit_A, const double minimum_voltage_V,
                                  const double assumed_power_consumption_W) {
  // The port is already used
  if (power_ports_[port_id] != nullptr) return -1;

  power_ports_[port_id] = new PowerPort(port_id, current_limit_A, minimum_voltage_V, assumed_power_consumption_W);
  return 0;
}

int PowerControlUnit::ClosePort(const int port_id) {
  // The port not used
  if (power_ports_[port_id] == nullptr) return -1;

  PowerPort* port = power_ports_.at(port_id);
  delete port;
  power_ports_.erase(port_id);
  return 0;
}

std::string PowerControlUnit::GetLogHeader() const {
  std::string str_tmp = "";
  return str_tmp;
}

std::string PowerControlUnit::GetLogValue() const {
  std::string str_tmp = "";
  return str_tmp;
}

}  // namespace s2e::components
