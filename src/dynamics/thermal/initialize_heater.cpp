/**
 * @file initialize_heater.cpp
 * @brief Initialize function for heater
 */

#include "initialize_heater.hpp"

#include <cassert>

/* Import heater properties by reading CSV File (heaters.csv)

[File Formats of heater.csv]
column 1: Heater_id(int, Use values larger than or equal to 1)
column 2: Power Rating (double, [W])
column 3: Lower threshold of control (double, [degC])
column 4: Upper threshold of control (double, [degC])

First row is for Header, data begins from the second row
*/

Heater InitHeater(const std::vector<std::string>& heater_str) {
  using std::stod;
  using std::stoi;

  size_t heater_str_size_defined = 4;                    // Correct size of heater_str
  assert(heater_str.size() == heater_str_size_defined);  // Check if size of heater_str is correct

  int heater_id = 0;
  double power_rating_W = 0;  // [W]

  // Index to read from heater_str for each parameter
  int index_heater_id = 0;
  int index_power_rating = 1;

  heater_id = stoi(heater_str[index_heater_id]);
  power_rating_W = stod(heater_str[index_power_rating]);

  Heater heater(heater_id, power_rating_W);
  return heater;
}

HeaterController InitHeaterController(const std::vector<std::string>& heater_str) {
  using std::stod;

  size_t heater_str_size_defined = 4;                    // Correct size of heater_str
  assert(heater_str.size() == heater_str_size_defined);  // Check if size of heater_str is correct

  // Index to read from heater_str for each parameter
  int index_lower_threshold = 2;
  int index_upper_threshold = 3;

  double lower_threshold_degC = 0;  // [degC]
  double upper_threshold_degC = 0;  // [degC]

  lower_threshold_degC = stod(heater_str[index_lower_threshold]);
  upper_threshold_degC = stod(heater_str[index_upper_threshold]);

  HeaterController heater_controller(lower_threshold_degC, upper_threshold_degC);
  return heater_controller;
}
