/**
 * @file initialize_heater.hpp
 * @brief Initialize function for heater and heater controller
 */

#ifndef S2E_DYNAMICS_THERMAL_INITIALIZE_HEATER_HPP_
#define S2E_DYNAMICS_THERMAL_INITIALIZE_HEATER_HPP_

#include "heater.hpp"
#include "heater_controller.hpp"

/**
 * @fn InitHeater
 * @brief Initialize Heater object from csv file
 * @param[in] heater_str: str read from csv file
 * @return Heater
 */
Heater InitHeater(const std::vector<std::string>& heater_str);
/**
 * @fn InitHeaterController
 * @brief Initialize HeaterController object from csv file
 * @param[in] heater_str: str read from csv file
 * @return HeaterController
 */
HeaterController InitHeaterController(const std::vector<std::string>& heater_str);

#endif  // S2E_DYNAMICS_THERMAL_INITIALIZE_HEATER_HPP_
