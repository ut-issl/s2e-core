/**
 * @file initialize_temperature.hpp
 * @brief Initialize function for temperature
 */

#ifndef S2E_DYNAMICS_THERMAL_INITIALIZE_TEMPERATURE_H_
#define S2E_DYNAMICS_THERMAL_INITIALIZE_TEMPERATURE_H_

#include "Temperature.h"
class Temperature;

Temperature* InitTemperature(const std::string ini_path, const double rk_prop_step_sec);

#endif  // S2E_DYNAMICS_THERMAL_INITIALIZE_TEMPERATURE_H_
