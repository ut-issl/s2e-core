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
  void UpdateTotalHeatload(void);

  // Output from this class
  double GetSolarHeatload(void) const;
  double GetInternalHeatload(void) const;
  double GetHeaterHeatload(void) const;
  double GetTotalHeatload(void) const;

  // Setter
  void SetTime(double t);
  void SetInternalHeatload(double internal);
  void SetHeaterHeatload(double heater);
  void SetSolarHeatload(double solar);
};

#endif  // S2E_DYNAMICS_THERMAL_HEATLOAD_HPP_
