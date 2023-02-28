/*
 * @file pcu_initial_study.hpp
 * @brief Component emulation of Power Control Unit for initial study of spacecraft project
 */

#ifndef S2E_COMPONENTS_REAL_POWER_PCU_INITIAL_STUDY_HPP_
#define S2E_COMPONENTS_REAL_POWER_PCU_INITIAL_STUDY_HPP_

#include <library/logger/loggable.hpp>
#include <vector>

#include "../../base/component.hpp"
#include "battery.hpp"
#include "solar_array_panel.hpp"

class PcuInitialStudy : public Component, public ILoggable {
 public:
  /**
   * @fn PcuInitialStudy
   * @brief Constructor
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] saps: Solar Array Panels
   * @param [in] battery: Battery
   * @param [in] component_step_time_s: Component step time [sec]
   */
  PcuInitialStudy(const int prescaler, ClockGenerator* clock_generator, const std::vector<SAP*> saps, Battery* battery, double component_step_time_s);
  /**
   * @fn PcuInitialStudy
   * @brief Constructor
   * @param [in] clock_generator: Clock generator
   * @param [in] saps: Solar Array Panels
   * @param [in] battery: Battery
   */
  PcuInitialStudy(ClockGenerator* clock_generator, const std::vector<SAP*> saps, Battery* battery);
  /**
   * @fn ~PcuInitialStudy
   * @brief Destructor
   */
  ~PcuInitialStudy();

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

 private:
  const std::vector<SAP*> saps_;      //!< Solar Array Panels
  Battery* const battery_;            //!< Battery
  const double cc_charge_current_C_;  //!< Constant charge current [C]
  const double cv_charge_voltage_V_;  //!< Constant charge voltage [V]
  double bus_voltage_V_;              //!< Bus voltage [V]
  double power_consumption_W_;        //!< Power consumption [W]
  double compo_step_time_s_;          //!< Component step time [sec]

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(int time_count) override;

  /**
   * @fn CalcPowerConsumption
   * @brief Calculate power consumption
   * @param time_query: Time query
   */
  double CalcPowerConsumption(double time_query) const;

  /**
   * @fn UpdateChargeCurrentAndBusVoltage
   * @brief Update charge current and bus voltage
   */
  void UpdateChargeCurrentAndBusVoltage();
};

#endif  // S2E_COMPONENTS_REAL_POWER_PCU_INITIAL_STUDY_HPP_
