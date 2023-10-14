#include "heatload.hpp"

#include <cassert>
#include <cmath>
#include <environment/global/physical_constants.hpp>

using namespace std;
using namespace libra;

Heatload::Heatload(int node_id, std::vector<double> time_table_s, std::vector<double> internal_heatload_table_W)
    : node_id_(node_id), time_table_s_(time_table_s), internal_heatload_table_W_(internal_heatload_table_W) {
  elapsed_time_s_ = 0;
  elapsed_time_idx_ = 0;
  solar_heatload_W_ = 0.0;
  internal_heatload_W_ = 0.0;
  heater_heatload_W_ = 0.0;
  total_heatload_W_ = 0.0;

  AssertHeatloadParams();

  time_table_period_s_ = time_table_s_[time_table_s_.size() - 1];
  residual_elapsed_time_s_ = fmod(elapsed_time_s_, time_table_period_s_);
}

Heatload::~Heatload() {}

void Heatload::CalcInternalHeatload(void) {
  double time_table_lower_s = time_table_s_[elapsed_time_idx_];      // Time in time_table_s_ that is below and closest to elapsed_time_s_ [s]
  double time_table_upper_s = time_table_s_[elapsed_time_idx_ + 1];  // Time in time_table_s_ that is above and closest to elapsed_time_s_ [s]

  double internal_heatload_table_lower_W = internal_heatload_table_W_[elapsed_time_idx_];      // Internal heat value at time_table_lower_s [W]
  double internal_heatload_table_upper_W = internal_heatload_table_W_[elapsed_time_idx_ + 1];  // Internal heat value at time_table_upper_s [W]

  double time_ratio = (residual_elapsed_time_s_ - time_table_lower_s) / (time_table_upper_s - time_table_lower_s);  // ratio of interpolation
  double internal_heatload_W =
      internal_heatload_table_lower_W +
      time_ratio * (internal_heatload_table_upper_W - internal_heatload_table_lower_W);  // internal heatload calculated by interpolation [W]
  internal_heatload_W_ = internal_heatload_W;
}

void Heatload::SetElapsedTime_s(double elapsed_time_s) {
  elapsed_time_s_ = elapsed_time_s;
  residual_elapsed_time_s_ = fmod(elapsed_time_s_, time_table_period_s_);

  while (residual_elapsed_time_s_ > time_table_s_[elapsed_time_idx_ + 1]) {
    elapsed_time_idx_++;
    if (elapsed_time_idx_ > time_table_s_.size() - 1) {
      elapsed_time_idx_ = 0;
    }
  }
}

void Heatload::AssertHeatloadParams() {
  // size of time_table_s_ and internal_heatload_table_W_ must be larger than 1
  assert(time_table_s_.size() >= 1);
  assert(internal_heatload_table_W_.size() >= 1);
  // size of time_table_s_ and internal_heatload_table_W_ must be the same
  assert(time_table_s_.size() == internal_heatload_table_W_.size());
}

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

  assert(time_str.size() >= 2);                             // Number of columns must be larger than 2
  assert(internal_heatload_str.size() >= 2);                // Number of columns must be larger than 2
  assert(time_str.size() == internal_heatload_str.size());  // Number of columns must be same

  std::vector<double> time_table_s(time_str.size() - 1);                            // exclude index
  std::vector<double> internal_heatload_table_W(internal_heatload_str.size() - 1);  // exclude index

  int node_id = stoi(internal_heatload_str[0]);  // First data of internal_heatload_str is node id

  // read table
  for (size_t i = 0; i < time_str.size() - 1; ++i) {
    time_table_s[i] = stod(time_str[i + 1]);
    internal_heatload_table_W[i] = stod(internal_heatload_str[i + 1]);
  }

  return Heatload(node_id, time_table_s, internal_heatload_table_W);
}
