/**
 * @file initialize_heatload.hpp
 * @brief Initialize function for heatload
 */

#ifndef S2E_DYNAMICS_THERMAL_INITIALIZE_HEATLOAD_HPP_
#define S2E_DYNAMICS_THERMAL_INITIALIZE_HEATLOAD_HPP_

#include "heatload.hpp"

/**
 * @fn InitHeatload
 * @brief Intialize Heatload object from csv file
 * @param[in] time_str: str representing time table, read from csv file
 * @param[in] internal_heatload_str: str representing internal heatload table, read from csv file
 * @return Heatload
 */
Heatload InitHeatload(const std::vector<std::string>& time_str, const std::vector<std::string>& internal_heatload_str);

#endif  // S2E_DYNAMICS_THERMAL_INITIALIZE_HEATLOAD_HPP_
