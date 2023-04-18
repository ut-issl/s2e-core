#include "heatload.hpp"

#include <cmath>
#include <environment/global/physical_constants.hpp>

using namespace std;
using namespace libra;

Heatload::Heatload(int node_id, std::vector<double> times, std::vector<double> internal_values)
  : node_id_(node_id),
    times_(times),
    internal_values_(internal_values) {
  t_ = 0;
  idx_ = 0;
  solar_ = 0.0;
  internal_ = 0.0;
  heater_ = 0.0;
  total_ = 0.0;

  time_end_ = times_[times_.size() - 1];
  t_mod_ = fmod(t_, time_end_);
}

Heatload::~Heatload() {}

void Heatload::CalcInternalHeatload(void) { 
  // Linear Interpolation to Calculate Internal Heat Load
  double t_lower = times_[idx_];
  double t_upper = times_[idx_ + 1];
  
  double heat_lower = internal_values_[idx_];
  double heat_upper = internal_values_[idx_ + 1];

  double t_ratio = (t_mod_ - t_lower) / (t_upper - t_lower);
  double heat = heat_lower + t_ratio * (heat_upper - heat_lower);
  internal_ = heat;
}

void Heatload::UpdateTotalHeatload(void) { total_ = solar_ + internal_ + heater_; }

double Heatload::GetSolarHeatload(void) const { return solar_; }

double Heatload::GetInternalHeatload(void) const { return internal_; }

double Heatload::GetHeaterHeatload(void) const { return heater_; }

double Heatload::GetTotalHeatload(void) const { return total_; }

void Heatload::SetTime(double t) { 
  t_ = t;
  t_mod_ = fmod(t_, time_end_);

  while (t_mod_ > times_[idx_ + 1]) {
    idx_++;
    if (idx_ > times_.size() - 1) {
      idx_ = 0;
    }
  }
}

void Heatload::SetInternalHeatload(double internal) { internal_ = internal; }

void Heatload::SetSolarHeatload(double solar) { solar_ = solar; }

void Heatload::SetHeaterHeatload(double heater) { heater_ = heater; }
