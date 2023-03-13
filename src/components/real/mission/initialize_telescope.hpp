/*
 * @file initialize_telescope.hpp
 * @brief Initialize function of Telescope
 */

#ifndef S2E_COMPONENTS_REAL_MISSION_INITIALIZE_TELESCOPE_HPP_
#define S2E_COMPONENTS_REAL_MISSION_INITIALIZE_TELESCOPE_HPP_

#include "telescope.hpp"

/*
 * @fn InitTelescope
 * @brief Initialize function of Telescope
 * @param [in] clock_generator: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] file_name: Path to initialize file
 * @param [in] attitude: Attitude information
 * @param [in] hipparcos: Star information by Hipparcos catalogue
 * @param [in] local_celestial_information: Local celestial information
 */
Telescope InitTelescope(ClockGenerator* clock_generator, int sensor_id, const std::string file_name, const Attitude* attitude,
                        const HipparcosCatalogue* hipparcos, const LocalCelestialInformation* local_celestial_information);

#endif  // S2E_COMPONENTS_REAL_MISSION_INITIALIZE_TELESCOPE_HPP_
