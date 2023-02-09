/*
 * @file initialize_ground_station_calculator.hpp.hpp
 * @brief Initialize function for Ground Station Calculator
 */

#ifndef S2E_COMPONENTS_COMMUNICATION_INITIALIZE_GROUND_STATION_CALCULATOR_HPP_
#define S2E_COMPONENTS_COMMUNICATION_INITIALIZE_GROUND_STATION_CALCULATOR_HPP_

#include <components/communication/ground_station_calculator.hpp>

/*
 * @fn InitGscalculator
 * @brief Initialize function for Ground Station Calculator
 * @param [in] fname: Path to initialize file
 */

GScalculator InitGScalculator(const std::string fname);

#endif  // S2E_COMPONENTS_COMMUNICATION_INITIALIZE_GROUND_STATION_CALCULATOR_HPP_
