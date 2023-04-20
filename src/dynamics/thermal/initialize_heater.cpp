/**
 * @file initialize_heater.cpp
 * @brief Initialize function for heater
 */

#include "initialize_heater.hpp"

Heater InitHeater(const std::vector<std::string>& heater_str) {
  using std::stod;
  using std::stoi;

  int heater_id = 0;                    // heater number
  double power = 0;                     // [W]
  double lower_thershold = 0;           // [degC]
  double upper_threshold = 0;           // [degC]

  heater_id = stoi(heater_str[0]);        // column 1
  power = stod(heater_str[1]);            // column 2
  lower_thershold = stod(heater_str[2]);  // column 3
  upper_threshold = stod(heater_str[3]);  // column 4

  Heater heater(heater_id, power, lower_thershold, upper_threshold);
  return heater;
}
