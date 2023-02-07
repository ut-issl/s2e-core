/**
 *@file initialize_gnss_satellites.hpp
 *@brief Initialize functions for GnssSatellites class
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_INITIALIZE_GNSS_SATELLITES_H_
#define S2E_ENVIRONMENT_GLOBAL_INITIALIZE_GNSS_SATELLITES_H_

#include <environment/global/gnss_satellites.hpp>

/**
 *@fn InitGnssSatellites
 *@brief Initialize function for GnssSatellites class
 *@param [in] file_name: Path to the initialize function
 */
GnssSatellites* InitGnssSatellites(std::string file_name);

#endif  // S2E_ENVIRONMENT_GLOBAL_INITIALIZE_GNSS_SATELLITES_H_
