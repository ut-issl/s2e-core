/*
 * @file initialize_ground_station_calculator.hpp.hpp
 * @brief Initialize function for Ground Station Calculator
 */

#ifndef S2E_COMPONENTS_REAL_COMMUNICATION_INITIALIZE_GROUND_STATION_CALCULATOR_HPP_
#define S2E_COMPONENTS_REAL_COMMUNICATION_INITIALIZE_GROUND_STATION_CALCULATOR_HPP_

#include <components/real/communication/ground_station_calculator.hpp>

/*
 * @fn InitGscalculator
 * @brief Initialize function for Ground Station Calculator
 * @param [in] file_name: Path to initialize file
 */

GroundStationCalculator InitGScalculator(const std::string file_name);

#endif  // S2E_COMPONENTS_REAL_COMMUNICATION_INITIALIZE_GROUND_STATION_CALCULATOR_HPP_
