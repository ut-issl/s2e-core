/*
 * @file initialize_solar_array_panel.hpp
 * @brief Initialize function of SolarArrayPanel (Solar Array Panel)
 */

#ifndef S2E_COMPONENTS_REAL_POWER_INITIALIZE_SOLAR_ARRAY_PANEL_HPP_
#define S2E_COMPONENTS_REAL_POWER_INITIALIZE_SOLAR_ARRAY_PANEL_HPP_

#include <components/real/power/solar_array_panel.hpp>

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

#endif  // S2E_COMPONENTS_REAL_POWER_INITIALIZE_SOLAR_ARRAY_PANEL_HPP_
