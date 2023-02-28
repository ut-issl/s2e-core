/*
 * @file initialize_solar_array_panel.hpp
 * @brief Initialize function of SAP (Solar Array Panel)
 */

#ifndef S2E_COMPONENTS_REAL_POWER_INITIALIZE_SOLAR_ARRAY_PANEL_HPP_
#define S2E_COMPONENTS_REAL_POWER_INITIALIZE_SOLAR_ARRAY_PANEL_HPP_

#include <components/real/power/solar_array_panel.hpp>

/*
 * @fn InitSAP
 * @brief Initialize function of BAT
 * @param [in] clock_generator: Clock generator
 * @param [in] sap_id: SAP ID
 * @param [in] file_name: Path to initialize file
 * @param [in] srp: Solar Radiation Pressure environment
 * @param [in] local_celestial_information: Local celestial information
 * @param [in] compo_step_time: Component step time [sec]
 */
SAP InitSAP(ClockGenerator* clock_generator, int sap_id, const std::string file_name, const SolarRadiationPressureEnvironment* srp,
            const LocalCelestialInformation* local_celestial_information, double compo_step_time);

/*
 * @fn InitSAP
 * @brief Initialize function of BAT
 * @param [in] clock_generator: Clock generator
 * @param [in] sap_id: SAP ID
 * @param [in] file_name: Path to initialize file
 * @param [in] srp: Solar Radiation Pressure environment
 * @param [in] compo_step_time: Component step time [sec]
 */
SAP InitSAP(ClockGenerator* clock_generator, int sap_id, const std::string file_name, const SolarRadiationPressureEnvironment* srp,
            double compo_step_time);

#endif  // S2E_COMPONENTS_REAL_POWER_INITIALIZE_SOLAR_ARRAY_PANEL_HPP_
