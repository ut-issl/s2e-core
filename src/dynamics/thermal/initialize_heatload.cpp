/**
 * @file initialize_heatload.cpp
 * @brief Initialize function for heatload
 */

#include "initialize_heatload.hpp"

#include <cassert>

/* Import heatload properties by reading CSV File (heatload.csv)

[File Formats of heatload.csv]
First Row: Array of Simulation Times [s]
After Second Row:
  First Column: Node ID
  After Second Column: Array of Heatload Values [W]

Heatload object calculates heatload value of certain time by interpolating values of the heatload array

Data Example:
NodeID/Time, 0, 99, 100, 300, 500
0,          10, 10,  20,  10, 20

In this example, heatload at time 200s is calculated as 15W from interpolation

The number of nodes and heatloads must be the same

*/

Heatload InitHeatload(const std::vector<std::string>& time_str, const std::vector<std::string>& internal_heatload_str) {
  using std::stod;
  using std::stoi;

  assert(time_str.size() >= 2);                                                     // Number of columns must be larger than 2
  assert(internal_heatload_str.size() >= 2);                                        // Number of columns must be larger than 2
  assert(time_str.size() == internal_heatload_str.size());                          // Number of columns must be same

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
