/**
 * @file initialize_heatload.cpp
 * @brief Initialize function for heatload
 */

#include "initialize_heatload.hpp"

// [FIXME] Write discription of csv file for heatload initialization

Heatload InitHeatload(const std::vector<std::string>& time_str, const std::vector<std::string>& internal_heatload_str) {
  using std::stod;
  using std::stoi;

  // [FIXME] Include assertion for size of time_str and internal_heatload_str to be larger than 2
  std::vector<double> time_table_s(time_str.size() - 1);                            // exclude index
  std::vector<double> internal_heatload_table_W(internal_heatload_str.size() - 1);  // exclude index

  int node_id = stoi(internal_heatload_str[0]);                                     // First data of internal_heatload_str is node id

  // read table
  for (unsigned int i = 0; i < time_str.size() - 1; ++i) {
    time_table_s[i] = stod(time_str[i + 1]);
    internal_heatload_table_W[i] = stod(internal_heatload_str[i + 1]);
  }

  return Heatload(node_id, time_table_s, internal_heatload_table_W);
}
