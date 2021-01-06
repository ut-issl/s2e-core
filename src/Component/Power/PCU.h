#pragma once

#include "../Abstract/ComponentBase.h"
#include "../../Interface/LogOutput/ILoggable.h"
#include "../../Interface/SpacecraftInOut/Ports/PowerPort.h"
#include <map>

class PCU : public ComponentBase, public ILoggable
{
public:
  // Constructor/Destractor
  PCU(ClockGenerator* clock_gen);
  PCU(int prescaler, ClockGenerator* clock_gen);
  ~PCU();
  // Override ComponentBase
  void MainRoutine(int count) override;
  // Override ILoggable
  string GetLogHeader() const override;
  string GetLogValue() const override;

  // Getter
  inline PowerPort* GetPowerPort(int port_id){return ports_[port_id];};

  // Port control functions
  int ConnectPort(const int port_id, const double current_Limit, const double minimum_voltage, const double assumed_power_consumption);
  int ClosePort(const int port_id);

private:
  std::map<int, PowerPort*> ports_;
};