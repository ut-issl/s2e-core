/**
 * @file initialize_heater.cpp
 * @brief Initialize function for heater
 */

#include "initialize_heater.hpp"

Heater InitHeater(const std::vector<std::string>& heater_str) {
  using std::stod;
  using std::stoi;

  int heater_id = 0;  // heater number
  double power = 0;   // [W]

  heater_id = stoi(heater_str[0]);  // column 1
  power = stod(heater_str[1]);      // column 2

  Heater heater(heater_id, power);
  return heater;
}

HeaterController InitHeaterController(const std::vector<std::string>& heater_str) {
  using std::stod;

  double lower_threshold = 0;  // [degC]
  double upper_threshold = 0;  // [degC]

  lower_threshold = stod(heater_str[2]);  // column 3
  upper_threshold = stod(heater_str[3]);  // column 4

  HeaterController heater_controller(lower_threshold, upper_threshold);
  return heater_controller;
}
