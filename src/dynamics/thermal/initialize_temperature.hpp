/**
 * @file initialize_temperature.hpp
 * @brief Initialize function for temperature
 */

#ifndef S2E_DYNAMICS_THERMAL_INITIALIZE_TEMPERATURE_HPP_
#define S2E_DYNAMICS_THERMAL_INITIALIZE_TEMPERATURE_HPP_

#include "temperature.hpp"

/**
 * @fn InitTemperature
 * @brief Initialize Temperature object from csv file
 * @param[in] file_name: Directory of thermal input files
 * @param[in] rk_prop_step_s: time step interval for temperature propagation integration
 * @return Temperature*
 */
Temperature* InitTemperature(const std::string file_name, const double rk_prop_step_s);

#endif  // S2E_DYNAMICS_THERMAL_INITIALIZE_TEMPERATURE_HPP_
