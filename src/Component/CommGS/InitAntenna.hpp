/*
 * @file InitAntenna.hpp
 * @brief Initialize function for Antenna
 */
#pragma once

#include <Component/CommGS/Antenna.hpp>

/*
 * @fn InitAntenna
 * @brief Initialize function for Antenna
 * @param [in] antenna_id: Antenna ID
 * @param [in] file_name: Path to initialize file
 */
Antenna InitAntenna(const int antenna_id, const std::string file_name);
