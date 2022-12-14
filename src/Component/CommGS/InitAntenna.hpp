/*
 * @file InitAntenna.hpp
 * @brief Initialize function for Antenna
 */
#pragma once

#include <Component/CommGS/Antenna.hpp>

/*
 * @fn InitAntenna
 * @brief Initialize function for Antenna
 * @param [in] ant_id: Antenna ID
 * @param [in] fname: Path to initialize file
 */
Antenna InitAntenna(int ant_id, const std::string fname);
