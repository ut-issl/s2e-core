#pragma once

#include <Interface/LogOutput/ILoggable.h>
#include <Interface/SpacecraftInOut/Ports/PowerPort.h>

#include <map>

#include "../Abstract/ComponentBase.h"

class PCU : public ComponentBase, public ILoggable {
 public:
  // Constructor/Destractor
  PCU(ClockGenerator* clock_gen);
  PCU(int prescaler, ClockGenerator* clock_gen);
  ~PCU();
  // Override ComponentBase
  void MainRoutine(int count) override;
  // Override ILoggable
  std::string GetLogHeader() const override;
  std::string GetLogValue() const override;

  // Getter
  inline PowerPort* GetPowerPort(int port_id) { return ports_[port_id]; };

  // Port control functions
  int ConnectPort(const int port_id, const double current_Limit);
  int ConnectPort(const int port_id, const double current_Limit,
                  const double minimum_voltage,
                  const double assumed_power_consumption);
  int ClosePort(const int port_id);

 private:
  std::map<int, PowerPort*> ports_;
};
