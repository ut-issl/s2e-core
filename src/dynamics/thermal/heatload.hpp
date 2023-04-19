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
  double time_now_;
  int node_id_;
  std::vector<double> time_table_;
  std::vector<double> heatload_table_;

  unsigned int time_now_idx_;
  double solar_;
  double internal_;
  double heater_;
  double total_;

  double time_table_end_;
  double time_now_mod_;

 public:
  Heatload(const int node_id, const std::vector<double> time_table, const std::vector<double> heatload_table);
  virtual ~Heatload();

  void CalcInternalHeatload(void);

  void UpdateTotalHeatload(void) { total_ = solar_ + internal_ + heater_; }

  // Getter
  inline double GetSolarHeatload(void) const { return solar_; }
  inline double GetInternalHeatload(void) const { return internal_; }
  inline double GetHeaterHeatload(void) const { return heater_; }
  inline double GetTotalHeatload(void) const { return total_; }

  // Setter
  void SetTime(double t);
  inline void SetInternalHeatload(double internal) { internal_ = internal; }
  inline void SetSolarHeatload(double solar) { solar_ = solar; }
  inline void SetHeaterHeatload(double heater) { heater_ = heater; }
};

#endif  // S2E_DYNAMICS_THERMAL_HEATLOAD_HPP_
