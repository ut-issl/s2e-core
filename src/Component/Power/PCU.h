/*
 * @file PCU.h
 * @brief Component emulation of Power Control Unit
 */
#pragma once

#include <Interface/LogOutput/ILoggable.h>
#include <Interface/SpacecraftInOut/Ports/PowerPort.h>

#include <map>

#include "../Abstract/ComponentBase.h"

/*
 * @class PCU
 * @brief Component emulation of Power Control Unit
 */
class PCU : public ComponentBase, public ILoggable {
 public:
  /**
   * @fn PCU
   * @brief Constructor
   * @param [in] clock_gen: Clock generator
   */
  PCU(ClockGenerator* clock_gen);
  /**
   * @fn PCU
   * @brief Constructor
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   */
  PCU(int prescaler, ClockGenerator* clock_gen);
  /**
   * @fn ~PCU
   * @brief Destructor
   */
  ~PCU();

  // Override functions for ComponentBase
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(int count) override;

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  std::string GetLogHeader() const override;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  std::string GetLogValue() const override;

  /**
   * @fn GetPowerPort
   * @brief Return power port information
   * @param port_id: Power port ID
   */
  inline PowerPort* GetPowerPort(int port_id) { return ports_[port_id]; };

  // Port control functions
  /**
   * @fn ConnectPort
   * @brief Connect power port between components and PCU
   * @param port_id: Power port ID
   * @param [in] current_Limit: Threshold to detect over current [A]
   * @return 0: Success, -1: Error
   */
  int ConnectPort(const int port_id, const double current_Limit);
  /**
   * @fn ConnectPort
   * @brief Connect power port between components and PCU
   * @param port_id: Power port ID
   * @param [in] current_Limit: Threshold to detect over current [A]
   * @param [in] minimum_voltage: Minimum voltage to work the component [V]
   * @param [in] assumed_power_consumption: Assumed power consumption of the component [W]
   * @return 0: Success, -1: Error
   */
  int ConnectPort(const int port_id, const double current_Limit, const double minimum_voltage, const double assumed_power_consumption);
  /**
   * @fn ClosePort
   * @brief Close power port between components and PCU
   * @param port_id: Power port ID
   * @return 0: Success, -1: Error
   */
  int ClosePort(const int port_id);

 private:
  std::map<int, PowerPort*> ports_;  //!< Power port list
};
