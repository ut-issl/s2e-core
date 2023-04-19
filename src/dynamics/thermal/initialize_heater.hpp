/**
 * @file initialize_heater.hpp
 * @brief Initialize function for heater
 */

#ifndef S2E_DYNAMICS_THERMAL_INITIALIZE_HEATER_HPP_
#define S2E_DYNAMICS_THERMAL_INITIALIZE_HEATER_HPP_

#include "heater.hpp"

Heater InitHeater(const std::vector<std::string>& heater_str);

#endif  // S2E_DYNAMICS_THERMAL_INITIALIZE_HEATER_HPP_
