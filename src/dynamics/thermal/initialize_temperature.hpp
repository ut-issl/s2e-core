/**
 * @file initialize_temperature.hpp
 * @brief Initialize function for temperature
 */

#ifndef S2E_DYNAMICS_THERMAL_INITIALIZE_TEMPERATURE_HPP_
#define S2E_DYNAMICS_THERMAL_INITIALIZE_TEMPERATURE_HPP_

#include "temperature.hpp"

Temperature* InitTemperature(const std::string file_name, const double rk_prop_step_sec);

#endif  // S2E_DYNAMICS_THERMAL_INITIALIZE_TEMPERATURE_HPP_
