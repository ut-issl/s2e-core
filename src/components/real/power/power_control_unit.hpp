/*
 * @file power_control_unit.hpp
 * @brief Component emulation of Power Control Unit
 */

#ifndef S2E_COMPONENTS_REAL_POWER_POWER_CONTROL_UNIT_HPP_
#define S2E_COMPONENTS_REAL_POWER_POWER_CONTROL_UNIT_HPP_

#include <components/ports/power_port.hpp>
#include <logger/loggable.hpp>
#include <map>

#include "../../base/component.hpp"

namespace s2e::components {

/*
 * @class PowerControlUnit
 * @brief Component emulation of Power Control Unit
 */
class PowerControlUnit : public Component, public logger::ILoggable {
 public:
  /**
   * @fn PowerControlUnit
   * @brief Constructor
   * @param [in] clock_generator: Clock generator
   */
  PowerControlUnit(environment::ClockGenerator* clock_generator);
  /**
   * @fn PowerControlUnit
   * @brief Constructor
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   */
  PowerControlUnit(int prescaler, environment::ClockGenerator* clock_generator);
  /**
   * @fn ~PowerControlUnit
   * @brief Destructor
   */
  ~PowerControlUnit();

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(const int time_count) override;

  // Override logger::ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of logger::ILoggable
   */
  std::string GetLogHeader() const override;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of logger::ILoggable
   */
  std::string GetLogValue() const override;

  /**
   * @fn GetPowerPort
   * @brief Return power port information
   * @param port_id: Power port ID
   */
  inline PowerPort* GetPowerPort(const int port_id) { return power_ports_.at(port_id); };
  inline const PowerPort* GetPowerPort(const int port_id) const { return power_ports_.at(port_id); };

  // Port control functions
  /**
   * @fn ConnectPort
   * @brief Connect power port between components and PowerControlUnit
   * @param port_id: Power port ID
   * @param [in] current_limit_A: Threshold to detect over current [A]
   * @return 0: Success, -1: Error
   */
  int ConnectPort(const int port_id, const double current_limit_A);
  /**
   * @fn ConnectPort
   * @brief Connect power port between components and PowerControlUnit
   * @param port_id: Power port ID
   * @param [in] current_limit_A: Threshold to detect over current [A]
   * @param [in] minimum_voltage_V: Minimum voltage to work the component [V]
   * @param [in] assumed_power_consumption_W: Assumed power consumption of the component [W]
   * @return 0: Success, -1: Error
   */
  int ConnectPort(const int port_id, const double current_limit_A, const double minimum_voltage_V, const double assumed_power_consumption_W);
  /**
   * @fn ClosePort
   * @brief Close power port between components and PowerControlUnit
   * @param port_id: Power port ID
   * @return 0: Success, -1: Error
   */
  int ClosePort(const int port_id);

 private:
  std::map<int, PowerPort*> power_ports_;  //!< Power port list
};

} // namespace s2e::components

#endif  // S2E_COMPONENTS_REAL_POWER_POWER_CONTROL_UNIT_HPP_
