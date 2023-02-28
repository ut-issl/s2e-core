/*
 * @file solar_array_panel.hpp
 * @brief Component emulation of Solar Array Panel
 */

#ifndef S2E_COMPONENTS_REAL_POWER_SOLAR_ARRAY_PANEL_HPP_
#define S2E_COMPONENTS_REAL_POWER_SOLAR_ARRAY_PANEL_HPP_

#include <environment/local/local_celestial_information.hpp>
#include <environment/local/solar_radiation_pressure_environment.hpp>
#include <library/logger/loggable.hpp>
#include <library/math/vector.hpp>

#include "../../base/component.hpp"

class SAP : public Component, public ILoggable {
 public:
  /**
   * @fn SAP
   * @brief Constructor with prescaler
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: SAP ID
   * @param [in] number_of_series: Number of series connected solar cells
   * @param [in] number_of_parallel: Number of parallel connected solar cells
   * @param [in] cell_area: Area of a solar cell [m2]
   * @param [in] normal_vector: Normal vector of SAP on the body fixed frame
   * @param [in] cell_efficiency: Power generation efficiency of solar cell
   * @param [in] transmission_efficiency: Efficiency of transmission to PCU
   * @param [in] srp: Solar Radiation Pressure environment
   * @param [in] local_celestial_information: Local celestial information
   * @param [in] component_step_time_s: Component step time [sec]
   */
  SAP(const int prescaler, ClockGenerator* clock_generator, int component_id, int number_of_series, int number_of_parallel, double cell_area,
      libra::Vector<3> normal_vector, double cell_efficiency, double transmission_efficiency, const SolarRadiationPressureEnvironment* srp,
      const LocalCelestialInformation* local_celestial_information, double component_step_time_s);
  /**
   * @fn SAP
   * @brief Constructor with prescaler
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: SAP ID
   * @param [in] number_of_series: Number of series connected solar cells
   * @param [in] number_of_parallel: Number of parallel connected solar cells
   * @param [in] cell_area: Area of a solar cell [m2]
   * @param [in] normal_vector: Normal vector of SAP on the body fixed frame
   * @param [in] cell_efficiency: Power generation efficiency of solar cell
   * @param [in] transmission_efficiency: Efficiency of transmission to PCU
   * @param [in] srp: Solar Radiation Pressure environment
   * @param [in] component_step_time_s: Component step time [sec]
   */
  SAP(const int prescaler, ClockGenerator* clock_generator, int component_id, int number_of_series, int number_of_parallel, double cell_area,
      libra::Vector<3> normal_vector, double cell_efficiency, double transmission_efficiency, const SolarRadiationPressureEnvironment* srp,
      double component_step_time_s);
  /**
   * @fn SAP
   * @brief Constructor without prescaler
   * @note prescaler is set as 10, compo_step_sec is set as
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: SAP ID
   * @param [in] number_of_series: Number of series connected solar cells
   * @param [in] number_of_parallel: Number of parallel connected solar cells
   * @param [in] cell_area: Area of a solar cell [m2]
   * @param [in] normal_vector: Normal vector of SAP on the body fixed frame
   * @param [in] cell_efficiency: Power generation efficiency of solar cell
   * @param [in] transmission_efficiency: Efficiency of transmission to PCU
   * @param [in] srp: Solar Radiation Pressure environment
   * @param [in] local_celestial_information: Local celestial information
   */
  SAP(ClockGenerator* clock_generator, int component_id, int number_of_series, int number_of_parallel, double cell_area,
      libra::Vector<3> normal_vector, double cell_efficiency, double transmission_efficiency, const SolarRadiationPressureEnvironment* srp,
      const LocalCelestialInformation* local_celestial_information);
  /**
   * @fn SAP
   * @brief Copy constructor
   */
  SAP(const SAP& obj);
  /**
   * @fn ~SAP
   * @brief Destructor
   */
  ~SAP();

  /**
   * @fn GetPowerGeneration
   * @brief Return power generation [W]
   */
  double GetPowerGeneration() const;
  /**
   * @fn SetVoltage_V
   * @brief Set voltage
   */
  void SetVoltage_V(const double voltage);

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
  const int component_id_;                //!< SAP ID TODO: Use string?
  const int number_of_series_;            //!< Number of series connected solar cells
  const int number_of_parallel_;          //!< Number of parallel connected solar cells
  const double cell_area_m2_;             //!< Solar cell area [m^2]
  const libra::Vector<3> normal_vector_;  //!< Normal vector of SAP on the body fixed frame
  const double cell_efficiency_;          //!< Power generation efficiency of solar cell
  const double transmission_efficiency_;  //!< Efficiency of transmission to PCU

  const SolarRadiationPressureEnvironment* const srp_environment_;  //!< Solar Radiation Pressure environment
  const LocalCelestialInformation* local_celestial_information_;    //!< Local celestial information

  double voltage_V_;           //!< Voltage [V]
  double power_generation_W_;  //!< Generated power [W]

  static const double solar_constant_;  //!< Solar constant TODO: Use SolarRadiationPressureEnvironment?
  static const double light_speed_;     //!< Speed of light TODO: Use PhysicalConstant?
  double compo_step_time_s_;            //!< Component step time [sec]

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(const int time_count) override;
};

#endif  // S2E_COMPONENTS_REAL_POWER_SOLAR_ARRAY_PANEL_HPP_
