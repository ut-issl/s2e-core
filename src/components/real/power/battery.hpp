/*
 * @file battery.hpp
 * @brief Component emulation of battery
 */

#ifndef S2E_COMPONENTS_REAL_POWER_BATTERY_HPP_P_
#define S2E_COMPONENTS_REAL_POWER_BATTERY_HPP_P_

#include <logger/loggable.hpp>
#include <vector>

#include "../../base/component.hpp"

namespace s2e::components {

/*
 * @class Battery
 * @brief Component emulation of battery
 */
class Battery : public Component, public logger::ILoggable {
 public:
  /**
   * @fn Battery
   * @brief Constructor with prescaler
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] number_of_series: Number of series connected cells
   * @param [in] number_of_parallel: Number of parallel connected cells
   * @param [in] cell_capacity_Ah: Power capacity of a cell [Ah]
   * @param [in] cell_discharge_curve_coefficients: Discharge curve coefficients for a cell
   * @param [in] initial_dod: Initial depth of discharge
   * @param [in] cc_charge_c_rate: Constant charge rate [C]
   * @param [in] cv_charge_voltage_V: Constant charge voltage [V]
   * @param [in] battery_resistance_Ohm: Battery internal resistance [Ohm]
   * @param [in] component_step_time_s: Component step time [sec]
   */
  Battery(const int prescaler, environment::ClockGenerator* clock_generator, int number_of_series, int number_of_parallel, double cell_capacity_Ah,
          const std::vector<double> cell_discharge_curve_coefficients, double initial_dod, double cc_charge_c_rate, double cv_charge_voltage_V,
          double battery_resistance_Ohm, double component_step_time_s);
  /**
   * @fn Battery
   * @brief Constructor without prescaler
   * @note prescaler is set as 10
   * @param [in] clock_generator: Clock generator
   * @param [in] number_of_series: Number of series connected cells
   * @param [in] number_of_parallel: Number of parallel connected cells
   * @param [in] cell_capacity_Ah: Power capacity of a cell [Ah]
   * @param [in] cell_discharge_curve_coefficients: Discharge curve coefficients for a cell
   * @param [in] initial_dod: Initial depth of discharge
   * @param [in] cc_charge_c_rate: Constant charge current [C]
   * @param [in] cv_charge_voltage_V: Constant charge voltage [V]
   * @param [in] battery_resistance_Ohm: Battery internal resistance [Ohm]
   */
  Battery(environment::ClockGenerator* clock_generator, int number_of_series, int number_of_parallel, double cell_capacity_Ah,
          const std::vector<double> cell_discharge_curve_coefficients, double initial_dod, double cc_charge_c_rate, double cv_charge_voltage_V,
          double battery_resistance_Ohm);
  /**
   * @fn Battery
   * @brief Copy constructor
   */
  Battery(const Battery& obj);
  /**
   * @fn ~Battery
   * @brief Destructor
   */
  ~Battery();

  /**
   * @fn SetChargeCurrent
   * @brief Set charge current [A]
   */
  inline void SetChargeCurrent(const double current_A) { charge_current_A_ = current_A; }

  /**
   * @fn GetVoltage_V
   * @brief Return battery voltage [V]
   */
  inline double GetVoltage_V() const { return battery_voltage_V_; }

  /**
   * @fn GetResistance_Ohm
   * @brief Return battery resistance [Ohm]
   */
  inline double GetResistance_Ohm() const { return battery_resistance_Ohm_; }

  /**
   * @fn GetCcChargeCurrent_C
   * @brief Return constant charge current [C]
   * @note TODO: Change implementation?
   */
  inline double GetCcChargeCurrent_C() const { return cc_charge_current_C_; }

  /**
   * @fn GetCvChargeVoltage_V
   * @brief Return constant charge voltage [V]
   * @note TODO: Change implementation?
   */
  inline double GetCvChargeVoltage_V() const { return cv_charge_voltage_V_; }

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

 private:
  const int number_of_series_;                                   //!< Number of series connected cells
  const int number_of_parallel_;                                 //!< Number of parallel connected cells
  const double cell_capacity_Ah_;                                //!< Power capacity of a cell [Ah]
  const std::vector<double> cell_discharge_curve_coefficients_;  //!< Discharge curve coefficients for a cell
  const double cc_charge_current_C_;                             //!< Constant charge current [C]
  const double cv_charge_voltage_V_;                             //!< Constant charge voltage [V]
  double battery_voltage_V_;                                     //!< Battery voltage [V]
  double depth_of_discharge_percent_;                            //!< Depth of discharge [%]
  double charge_current_A_;                                      //!< Charge current [A]
  double battery_resistance_Ohm_;                                //!< Battery internal resistance [Ohm]
  double compo_step_time_s_;                                     //!< Component step time [sec]

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(const int time_count) override;

  /**
   * @fn UpdateBatVoltage
   * @brief Calculate battery voltage
   */
  void UpdateBatVoltage();
};

/*
 * @fn InitBAT
 * @brief Initialize function of Battery
 * @param [in] clock_generator: Clock generator
 * @param [in] bat_id: Battery ID
 * @param [in] file_name: Path to initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 */
Battery InitBAT(environment::ClockGenerator* clock_generator, int bat_id, const std::string file_name, double component_step_time_s);

} // namespace s2e::components

#endif  // S2E_COMPONENTS_REAL_POWER_BATTERY_HPP_P_
