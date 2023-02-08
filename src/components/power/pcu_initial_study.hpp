/*
 * @file pcu_initial_study.hpp
 * @brief Component emulation of Power Control Unit for initial study of spacecraft project
 */

#ifndef S2E_COMPONENTS_POWER_PCU_INITIAL_STUDY_H_
#define S2E_COMPONENTS_POWER_PCU_INITIAL_STUDY_H_

#include <interface/log_output/loggable.hpp>
#include <vector>

#include "../base_classes/component_base.hpp"
#include "battery.hpp"
#include "solar_array_paddle.hpp"

class PCU_InitialStudy : public ComponentBase, public ILoggable {
 public:
  /**
   * @fn PCU_InitialStudy
   * @brief Constructor
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] saps: Solar Array Panels
   * @param [in] bat: Battery
   * @param [in] compo_step_time: Component step time [sec]
   */
  PCU_InitialStudy(const int prescaler, ClockGenerator* clock_gen, const std::vector<SAP*> saps, BAT* bat, double compo_step_time);
  /**
   * @fn PCU_InitialStudy
   * @brief Constructor
   * @param [in] clock_gen: Clock generator
   * @param [in] saps: Solar Array Panels
   * @param [in] bat: Battery
   */
  PCU_InitialStudy(ClockGenerator* clock_gen, const std::vector<SAP*> saps, BAT* bat);
  /**
   * @fn ~PCU_InitialStudy
   * @brief Destructor
   */
  ~PCU_InitialStudy();

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
  const std::vector<SAP*> saps_;    //!< Solar Array Panels
  BAT* const bat_;                  //!< Battery
  const double cc_charge_current_;  //!< Constant charge current [C]
  const double cv_charge_voltage_;  //!< Constant charge voltage [V]
  double bus_voltage_;              //!< Bus voltage [V]
  double power_consumption_;        //!< Power consumption [W]
  double compo_step_time_;          //!< Component step time [sec]

  // Override functions for ComponentBase
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

#endif  // S2E_COMPONENTS_POWER_PCU_INITIAL_STUDY_H_
