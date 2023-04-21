/**
 * @file temperature.hpp
 * @brief Initialize temperature
 */

#ifndef S2E_DYNAMICS_THERMAL_TEMPERATURE_HPP_
#define S2E_DYNAMICS_THERMAL_TEMPERATURE_HPP_

#include <library/logger/loggable.hpp>
#include <string>
#include <vector>

#include "heater.hpp"
#include "heatload.hpp"
#include "node.hpp"

enum class SolarCalcSetting {
  kEnable,
  kDisable,
};

class Temperature : public ILoggable {
 protected:
  std::vector<std::vector<double>> conductance_matrix_;  // Coupling of node i and node j by heat conduction
  std::vector<std::vector<double>> radiation_matrix_;    // Coupling of node i and node j by thermal radiation
  std::vector<Node> nodes_;                              // vector of nodes
  std::vector<Heatload> heatloads_;                      // vector of heatloads
  std::vector<Heater> heaters_;                          // vector of heaters
  std::vector<HeaterController> heater_controllers_;     // vector of heater controllers
  int node_num_;                                         // number of nodes
  double propagation_step_;                              // 積分刻み幅[sec]
  double propagation_time_;              // Temperatureクラス内での累積積分時間(end_timeに等しくなるまで積分する)
  bool is_calc_enabled_;                 // 温度更新をするかどうかのブーリアン
  SolarCalcSetting solar_calc_setting_;  // setting for solar calculation
  bool debug_;

  void RungeOneStep(double t, double dt, libra::Vector<3> sun_direction, int node_num);
  std::vector<double> OdeTemperature(std::vector<double> x, double t, const libra::Vector<3> sun_direction,
                                     int node_num);  // 温度に関する常微分方程式, xはnodeの温度をならべたもの

 public:
  Temperature(const std::vector<std::vector<double>> conductance_matrix, const std::vector<std::vector<double>> radiation_matrix,
              std::vector<Node> nodes, std::vector<Heatload> heatloads, std::vector<Heater> heaters, std::vector<HeaterController> heater_controllers,
              const int node_num, const double propstep, const bool is_calc_enabled, const SolarCalcSetting solar_calc_setting, const bool debug);
  Temperature();
  virtual ~Temperature();
  void Propagate(libra::Vector<3> sun_direction,
                 const double endtime);  // 太陽入熱量計算のため, 太陽方向の情報を入手

  inline std::vector<Node> GetNodes() { return nodes_; };
  double GetHeaterPower(int node_id);
  void UpdateHeaterStatus(void);
  std::string GetLogHeader() const;
  std::string GetLogValue() const;
  void PrintParams(void);  // デバッグ出力
};

#endif  // S2E_DYNAMICS_THERMAL_TEMPERATURE_HPP_
