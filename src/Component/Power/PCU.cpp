#include "PCU.h"

PCU::PCU(ClockGenerator* clock_gen) : ComponentBase(1, clock_gen) {}

PCU::PCU(int prescaler, ClockGenerator* clock_gen)
    : ComponentBase(prescaler, clock_gen) {}

PCU::~PCU() {}

void PCU::MainRoutine(int count) {
  double current_ = ports_[1]->GetCurrentConsumption();
}

int PCU::ConnectPort(const int port_id, const double current_Limit) {
  // The port is already used
  if (ports_[port_id] != nullptr) return -1;

  ports_[port_id] = new PowerPort(port_id, current_Limit);
  return 0;
}

int PCU::ConnectPort(const int port_id, const double current_Limit,
                     const double minimum_voltage,
                     const double assumed_power_consumption) {
  // The port is already used
  if (ports_[port_id] != nullptr) return -1;

  ports_[port_id] = new PowerPort(port_id, current_Limit, minimum_voltage,
                                  assumed_power_consumption);
  return 0;
}

int PCU::ClosePort(const int port_id) {
  // The port not used
  if (ports_[port_id] == nullptr) return -1;

  PowerPort* port = ports_.at(port_id);
  delete port;
  ports_.erase(port_id);
  return 0;
}

std::string PCU::GetLogHeader() const {
  std::string str_tmp = "";
  return str_tmp;
}

std::string PCU::GetLogValue() const {
  std::string str_tmp = "";
  return str_tmp;
}
