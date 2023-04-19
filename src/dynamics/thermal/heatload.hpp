/**
 * @file heatload.hpp
 * @brief heatload for thermal node
 */

#ifndef S2E_DYNAMICS_THERMAL_HEATLOAD_HPP_
#define S2E_DYNAMICS_THERMAL_HEATLOAD_HPP_

#include <library/logger/logger.hpp>
#include <string>
#include <vector>

class Heatload {
 protected:
  double t_;
  int node_id_;
  std::vector<double> times_;
  std::vector<double> internal_values_;

  int idx_;
  double solar_;
  double internal_;
  double heater_;
  double total_;

  double time_end_;
  double t_mod_;

 public:
  Heatload(const int node_id, const std::vector<double> times, const std::vector<double> internal_values);
  virtual ~Heatload();

  void CalcInternalHeatload(void);

  void Heatload::UpdateTotalHeatload(void) { total_ = solar_ + internal_ + heater_; }

  // Getter
  inline double Heatload::GetSolarHeatload(void) const { return solar_; }
  inline double Heatload::GetInternalHeatload(void) const { return internal_; }
  inline double Heatload::GetHeaterHeatload(void) const { return heater_; }
  inline double Heatload::GetTotalHeatload(void) const { return total_; }

  // Setter
  void SetTime(double t);
  inline void Heatload::SetInternalHeatload(double internal) { internal_ = internal; }
  inline void Heatload::SetSolarHeatload(double solar) { solar_ = solar; }
  inline void Heatload::SetHeaterHeatload(double heater) { heater_ = heater; }
};

#endif  // S2E_DYNAMICS_THERMAL_HEATLOAD_HPP_
