#include "heatload.hpp"

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

  // [FIXME] Include assertion for size of time_table_s_ and internal_heatload_table_W_ to be larger than 2
  // [FIXME] Include assertion for size of time_table_s_ and internal_heatload_table_W_ to be the same
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
