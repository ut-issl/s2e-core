/*
 * @file battery.hpp
 * @brief Component emulation of battery
 */

#ifndef S2E_COMPONENTS_POWER_BATTERY_HPP_P_
#define S2E_COMPONENTS_POWER_BATTERY_HPP_P_

#include <interface/log_output/loggable.hpp>
#include <vector>

#include "../base_classes/component_base.hpp"

/*
 * @class BAT
 * @brief Component emulation of battery
 */
class BAT : public ComponentBase, public ILoggable {
 public:
  /**
   * @fn BAT
   * @brief Constructor with prescaler
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] number_of_series: Number of series connected cells
   * @param [in] number_of_parallel: Number of parallel connected cells
   * @param [in] cell_capacity: Power capacity of a cell [Ah]
   * @param [in] cell_discharge_curve_coeffs: Discharge curve coefficients for a cell
   * @param [in] initial_dod: Initial depth of discharge
   * @param [in] cc_charge_c_rate: Constant charge current [C]
   * @param [in] cv_charge_voltage: Constant charge voltage [V]
   * @param [in] bat_resistance: Battery internal resistance [Ohm]
   * @param [in] compo_step_time: Component step time [sec]
   */
  BAT(const int prescaler, ClockGenerator* clock_gen, int number_of_series, int number_of_parallel, double cell_capacity,
      const std::vector<double> cell_discharge_curve_coeffs, double initial_dod, double cc_charge_c_rate, double cv_charge_voltage,
      double bat_resistance, double compo_step_time);
  /**
   * @fn BAT
   * @brief Constructor without prescaler
   * @note prescaler is set as 10
   * @param [in] clock_gen: Clock generator
   * @param [in] number_of_series: Number of series connected cells
   * @param [in] number_of_parallel: Number of parallel connected cells
   * @param [in] cell_capacity: Power capacity of a cell [Ah]
   * @param [in] cell_discharge_curve_coeffs: Discharge curve coefficients for a cell
   * @param [in] initial_dod: Initial depth of discharge
   * @param [in] cc_charge_c_rate: Constant charge current [C]
   * @param [in] cv_charge_voltage: Constant charge voltage [V]
   * @param [in] bat_resistance: Battery internal resistance [Ohm]
   * @param [in] compo_step_time: Component step time [sec]
   */
  BAT(ClockGenerator* clock_gen, int number_of_series, int number_of_parallel, double cell_capacity,
      const std::vector<double> cell_discharge_curve_coeffs, double initial_dod, double cc_charge_c_rate, double cv_charge_voltage,
      double bat_resistance);
  /**
   * @fn BAT
   * @brief Copy constructor
   */
  BAT(const BAT& obj);
  /**
   * @fn ~BAT
   * @brief Destructor
   */
  ~BAT();

  /**
   * @fn SetChargeCurrent
   * @brief Set charge current [A]
   */
  void SetChargeCurrent(const double current);

  /**
   * @fn GetBatVoltage
   * @brief Return battery voltage [V]
   */
  double GetBatVoltage() const;
  /**
   * @fn GetBatResistance
   * @brief Return battery resistance [Ohm]
   */
  double GetBatResistance() const;
  /**
   * @fn GetCCChargeCurrent
   * @brief Return constant charge current [C]
   * @note TODO: Change implementation?
   */
  double GetCCChargeCurrent() const;
  /**
   * @fn GetCVChargeVoltage
   * @brief Return constant charge voltage [V]
   * @note TODO: Change implementation?
   */
  double GetCVChargeVoltage() const;

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
  const int number_of_series_;                             //!< Number of series connected cells
  const int number_of_parallel_;                           //!< Number of parallel connected cells
  const double cell_capacity_;                             //!< Power capacity of a cell [Ah]
  const std::vector<double> cell_discharge_curve_coeffs_;  //!< Discharge curve coefficients for a cell
  const double cc_charge_current_;                         //!< Constant charge current [C]
  const double cv_charge_voltage_;                         //!< Constant charge voltage [V]
  double bat_voltage_;                                     //!< Battery voltage [V]
  double dod_;                                             //!< Depth of discharge [%]
  double charge_current_;                                  //!< Charge current [A]
  double bat_resistance_;                                  //!< Battery internal resistance [Ohm]
  double compo_step_time_;                                 //!< Component step time [sec]

  // Override functions for ComponentBase
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(int count) override;

  /**
   * @fn UpdateBatVoltage
   * @brief Calculate battery voltage
   */
  void UpdateBatVoltage();
};

#endif  // S2E_COMPONENTS_POWER_BATTERY_HPP_P_
