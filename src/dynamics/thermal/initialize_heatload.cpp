/**
 * @file initialize_heatload.cpp
 * @brief Initialize function for heatload
 */

#include "initialize_heatload.hpp"

Heatload InitHeatload(const std::vector<std::string>& time_str, const std::vector<std::string>& heatload_str) {
  using std::stod;
  using std::stoi;

  std::vector<double> times(time_str.size() - 1);          // exclude index
  std::vector<double> heatloads(heatload_str.size() - 1);  // exclude index

  int node_id = stoi(heatload_str[0]);  // First data is node id

  for (int i = 0; i < time_str.size() - 1; ++i) {
    times[i] = stod(time_str[i + 1]);
    heatloads[i] = stod(heatload_str[i + 1]);
  }

  return Heatload(node_id, times, heatloads);
}
