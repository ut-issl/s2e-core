/*
 * @file solar_array_panel.hpp
 * @brief Component emulation of Solar Array Panel
 */

#ifndef S2E_COMPONENTS_REAL_POWER_SOLAR_ARRAY_PANEL_HPP_
#define S2E_COMPONENTS_REAL_POWER_SOLAR_ARRAY_PANEL_HPP_

#include <environment/local/local_celestial_information.hpp>
#include <environment/local/solar_radiation_pressure_environment.hpp>
#include <logger/loggable.hpp>
#include <math_physics/math/vector.hpp>

#include "../../base/component.hpp"

namespace s2e::components {

class SolarArrayPanel : public Component, public ILoggable {
 public:
  /**
   * @fn SolarArrayPanel
   * @brief Constructor with prescaler
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: SolarArrayPanel ID
   * @param [in] number_of_series: Number of series connected solar cells
   * @param [in] number_of_parallel: Number of parallel connected solar cells
   * @param [in] cell_area_m2: Area of a solar cell [m2]
   * @param [in] normal_vector: Normal vector of SolarArrayPanel on the body fixed frame
   * @param [in] cell_efficiency: Power generation efficiency of solar cell
   * @param [in] transmission_efficiency: Efficiency of transmission to PCU
   * @param [in] srp_environment: Solar Radiation Pressure environment
   * @param [in] local_celestial_information: Local celestial information
   * @param [in] component_step_time_s: Component step time [sec]
   */
  SolarArrayPanel(const int prescaler, ClockGenerator* clock_generator, int component_id, int number_of_series, int number_of_parallel,
                  double cell_area_m2, s2e::math::Vector<3> normal_vector, double cell_efficiency, double transmission_efficiency,
                  const SolarRadiationPressureEnvironment* srp_environment, const LocalCelestialInformation* local_celestial_information,
                  double component_step_time_s);
  /**
   * @fn SolarArrayPanel
   * @brief Constructor with prescaler
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: SolarArrayPanel ID
   * @param [in] number_of_series: Number of series connected solar cells
   * @param [in] number_of_parallel: Number of parallel connected solar cells
   * @param [in] cell_area_m2: Area of a solar cell [m2]
   * @param [in] normal_vector: Normal vector of SolarArrayPanel on the body fixed frame
   * @param [in] cell_efficiency: Power generation efficiency of solar cell
   * @param [in] transmission_efficiency: Efficiency of transmission to PCU
   * @param [in] srp_environment: Solar Radiation Pressure environment
   * @param [in] component_step_time_s: Component step time [sec]
   */
  SolarArrayPanel(const int prescaler, ClockGenerator* clock_generator, int component_id, int number_of_series, int number_of_parallel,
                  double cell_area_m2, s2e::math::Vector<3> normal_vector, double cell_efficiency, double transmission_efficiency,
                  const SolarRadiationPressureEnvironment* srp_environment, double component_step_time_s);
  /**
   * @fn SolarArrayPanel
   * @brief Constructor without prescaler
   * @note prescaler is set as 10, compo_step_sec is set as 0.1
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id: SolarArrayPanel ID
   * @param [in] number_of_series: Number of series connected solar cells
   * @param [in] number_of_parallel: Number of parallel connected solar cells
   * @param [in] cell_area_m2: Area of a solar cell [m2]
   * @param [in] normal_vector: Normal vector of SolarArrayPanel on the body fixed frame
   * @param [in] cell_efficiency: Power generation efficiency of solar cell
   * @param [in] transmission_efficiency: Efficiency of transmission to PCU
   * @param [in] srp_environment: Solar Radiation Pressure environment
   * @param [in] local_celestial_information: Local celestial information
   */
  SolarArrayPanel(ClockGenerator* clock_generator, int component_id, int number_of_series, int number_of_parallel, double cell_area_m2,
                  s2e::math::Vector<3> normal_vector, double cell_efficiency, double transmission_efficiency,
                  const SolarRadiationPressureEnvironment* srp_environment, const LocalCelestialInformation* local_celestial_information);
  /**
   * @fn SolarArrayPanel
   * @brief Copy constructor
   */
  SolarArrayPanel(const SolarArrayPanel& obj);
  /**
   * @fn ~SolarArrayPanel
   * @brief Destructor
   */
  ~SolarArrayPanel();

  /**
   * @fn GetPowerGeneration_W
   * @brief Return power generation [W]
   */
  double GetPowerGeneration_W() const { return power_generation_W_; }

  /**
   * @fn SetVoltage_V
   * @brief Set voltage
   */
  void SetVoltage_V(const double voltage_V) { voltage_V_ = voltage_V; }

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
  const int component_id_;                //!< SolarArrayPanel ID TODO: Use string?
  const int number_of_series_;            //!< Number of series connected solar cells
  const int number_of_parallel_;          //!< Number of parallel connected solar cells
  const double cell_area_m2_;             //!< Solar cell area [m^2]
  const s2e::math::Vector<3> normal_vector_;   //!< Normal vector of SolarArrayPanel on the body fixed frame
  const double cell_efficiency_;          //!< Power generation efficiency of solar cell
  const double transmission_efficiency_;  //!< Efficiency of transmission to PCU

  const SolarRadiationPressureEnvironment* const srp_environment_;  //!< Solar Radiation Pressure environment
  const LocalCelestialInformation* local_celestial_information_;    //!< Local celestial information

  double voltage_V_;           //!< Voltage [V]
  double power_generation_W_;  //!< Generated power [W]

  double compo_step_time_s_;  //!< Component step time [sec]

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to calculate force generation
   */
  void MainRoutine(const int time_count) override;
};

/*
 * @fn InitSAP
 * @brief Initialize function of Battery
 * @param [in] clock_generator: Clock generator
 * @param [in] sap_id: SolarArrayPanel ID
 * @param [in] file_name: Path to initialize file
 * @param [in] srp_environment: Solar Radiation Pressure environment
 * @param [in] local_celestial_information: Local celestial information
 * @param [in] component_step_time_s: Component step time [sec]
 */
SolarArrayPanel InitSAP(ClockGenerator* clock_generator, int sap_id, const std::string file_name,
                        const SolarRadiationPressureEnvironment* srp_environment, const LocalCelestialInformation* local_celestial_information,
                        double component_step_time_s);

/*
 * @fn InitSAP
 * @brief Initialize function of Battery
 * @param [in] clock_generator: Clock generator
 * @param [in] sap_id: SolarArrayPanel ID
 * @param [in] file_name: Path to initialize file
 * @param [in] srp_environment: Solar Radiation Pressure environment
 * @param [in] component_step_time_s: Component step time [sec]
 */
SolarArrayPanel InitSAP(ClockGenerator* clock_generator, int sap_id, const std::string file_name,
                        const SolarRadiationPressureEnvironment* srp_environment, double component_step_time_s);

} // namespace s2e::components

#endif  // S2E_COMPONENTS_REAL_POWER_SOLAR_ARRAY_PANEL_HPP_
