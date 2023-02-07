/**
 *@file InitGnssSatellites.hpp
 *@brief Initialize functions for GnssSatellites class
 */

#pragma once

#include <environment/global/gnss_satellites.hpp>

/**
 *@fn InitGnssSatellites
 *@brief Initialize function for GnssSatellites class
 *@param [in] file_name: Path to the initialize function
 */
GnssSatellites* InitGnssSatellites(std::string file_name);
