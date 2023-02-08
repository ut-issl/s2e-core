/*
 * @file initialize_antenna.hpp
 * @brief Initialize function for Antenna
 */

#ifndef S2E_COMPONENTS_COMMUNICATION_INITIALIZE_ANTENNA_H_
#define S2E_COMPONENTS_COMMUNICATION_INITIALIZE_ANTENNA_H_

#include <components/communication/antenna.hpp>

/*
 * @fn InitAntenna
 * @brief Initialize function for Antenna
 * @param [in] antenna_id: Antenna ID
 * @param [in] file_name: Path to initialize file
 */
Antenna InitAntenna(const int antenna_id, const std::string file_name);

#endif  // S2E_COMPONENTS_COMMUNICATION_INITIALIZE_ANTENNA_H_
