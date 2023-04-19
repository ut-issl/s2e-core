#include "heatload.hpp"

#include <cmath>
#include <environment/global/physical_constants.hpp>

using namespace std;
using namespace libra;

Heatload::Heatload(int node_id, std::vector<double> times, std::vector<double> internal_values)
    : node_id_(node_id), time_table_(times), heatload_table_(internal_values) {
  time_now_ = 0;
  time_now_idx_ = 0;
  solar_ = 0.0;
  internal_ = 0.0;
  heater_ = 0.0;
  total_ = 0.0;

  time_table_end_ = time_table_[time_table_.size() - 1];
  time_now_mod_ = fmod(time_now_, time_table_end_);
}

Heatload::~Heatload() {}

void Heatload::CalcInternalHeatload(void) {
  // Linear Interpolation to Calculate Internal Heat Load
  double t_lower = time_table_[time_now_idx_];
  double t_upper = time_table_[time_now_idx_ + 1];

  double heat_lower = heatload_table_[time_now_idx_];
  double heat_upper = heatload_table_[time_now_idx_ + 1];

  double t_ratio = (time_now_mod_ - t_lower) / (t_upper - t_lower);
  double heat = heat_lower + t_ratio * (heat_upper - heat_lower);
  internal_ = heat;
}

void Heatload::SetTime(double t) {
  time_now_ = t;
  time_now_mod_ = fmod(time_now_, time_table_end_);

  while (time_now_mod_ > time_table_[time_now_idx_ + 1]) {
    time_now_idx_++;
    if (time_now_idx_ > time_table_.size() - 1) {
      time_now_idx_ = 0;
    }
  }
}
