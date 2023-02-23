/*
 * @file initialize_antenna.hpp
 * @brief Initialize function for Antenna
 */

#ifndef S2E_COMPONENTS_REAL_COMMUNICATION_INITIALIZE_ANTENNA_HPP_
#define S2E_COMPONENTS_REAL_COMMUNICATION_INITIALIZE_ANTENNA_HPP_

#include <components/real/communication/antenna.hpp>

/*
 * @fn InitAntenna
 * @brief Initialize function for Antenna
 * @param [in] antenna_id: Antenna ID
 * @param [in] file_name: Path to initialize file
 */
Antenna InitAntenna(const int antenna_id, const std::string file_name);

#endif  // S2E_COMPONENTS_REAL_COMMUNICATION_INITIALIZE_ANTENNA_HPP_
