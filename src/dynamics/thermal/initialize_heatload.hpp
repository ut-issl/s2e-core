/**
 * @file initialize_heatload.hpp
 * @brief Initialize function for heatload
 */

#ifndef S2E_DYNAMICS_THERMAL_INITIALIZE_HEATLOAD_HPP_
#define S2E_DYNAMICS_THERMAL_INITIALIZE_HEATLOAD_HPP_

#include "heatload.hpp"

Heatload InitHeatload(const std::vector<std::string>& time_str, const std::vector<std::string>& heatload_str);

#endif  // S2E_DYNAMICS_THERMAL_INITIALIZE_HEATLOAD_HPP_