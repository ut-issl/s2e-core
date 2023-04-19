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

class Temperature : public ILoggable {
 protected:
  std::vector<std::vector<double>> cij_;  // Coupling of node i and node j by heat conduction
  std::vector<std::vector<double>> rij_;  // Coupling of node i and node j by thermal radiation
  std::vector<Node> vnodes_;              // vector of nodes
  std::vector<Heatload> vheatloads_;      // vector of heatloads
  std::vector<Heater> vheaters_;          // vector of heaters
  int node_num_;                          // number of nodes
  double prop_step_;                      // 積分刻み幅[sec]
  double prop_time_;                      // Temperatureクラス内での累積積分時間(end_timeに等しくなるまで積分する)
  bool is_calc_enabled_;                  // 温度更新をするかどうかのブーリアン
  int heat_input_setting_;  // 各ノードの入熱量の計算方法 (0: csvは内部発熱のみ、1: csvは太陽入熱含めた総量)
  bool debug_;

  void RungeOneStep(double t, double dt, libra::Vector<3> sun_direction, int node_num);
  std::vector<double> OdeTemperature(std::vector<double> x, double t, const libra::Vector<3> sun_direction,
                                     int node_num);  // 温度に関する常微分方程式, xはnodeの温度をならべたもの

 public:
  Temperature(const std::vector<std::vector<double>> cij_, const std::vector<std::vector<double>> rij, std::vector<Node> vnodes,
              std::vector<Heatload> vheatloads, std::vector<Heater> vheaters, const int node_num, const double propstep, const bool is_calc_enabled,
              const int heat_input_setting, const bool debug);
  Temperature();
  virtual ~Temperature();
  void Propagate(libra::Vector<3> sun_direction,
                 const double endtime);  // 太陽入熱量計算のため, 太陽方向の情報を入手

  inline std::vector<Node> GetVnodes() { return vnodes_; };
  double GetHeaterPower(int node_id);
  void UpdateHeaterStatus(void);
  std::string GetLogHeader() const;
  std::string GetLogValue() const;
  void PrintParams(void);  // デバッグ出力
};

#endif  // S2E_DYNAMICS_THERMAL_TEMPERATURE_HPP_
