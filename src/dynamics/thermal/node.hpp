/**
 * @file node.hpp
 * @brief thermal calculation node
 */

#ifndef S2E_DYNAMICS_THERMAL_NODE_HPP_
#define S2E_DYNAMICS_THERMAL_NODE_HPP_

#include <environment/global/physical_constants.hpp>
#include <library/logger/logger.hpp>
#include <string>
#include <vector>

enum class NodeType { kDiffusive, kBoundary, kArithmetic };

class Node {
 protected:
  int node_id_;
  std::string node_name_;
  int heater_node_id_;  // heater node番号
  double temperature_;  // 温度[K]
  double capacity_;     // 熱容量[J/K]
  double alpha_rad_;
  double area_;                  // 太陽熱が入射する面の面積[m^2]
  libra::Vector<3> normal_v_b_;  // 太陽熱が入射する面の法線ベクトル(機体固定座標系)
  double solar_radiation_;       // 入射する太陽輻射熱[W]([J]に変換するためには時間をかけないといけないことに注意
  NodeType node_type_;           // ノードの種類 (0: diffusive, 1: boundary, 2: arithmetic)

 public:
  Node(const int node_id, const std::string node_name, const NodeType node_type, const int heater_node_id, const double temperature_ini,
       const double capacity_ini, const double alpha, const double area, libra::Vector<3> normal_v_b);
  virtual ~Node();

  // 熱計算用関数
  double CalcSolarRadiation(libra::Vector<3> sun_direction);  // 太陽入射熱を計算

  // Output from this class
  inline int GetNodeId(void) const { return node_id_; }
  inline std::string GetNodeLabel(void) const { return node_name_; }
  inline int GetHeaterNodeId(void) const { return heater_node_id_; }
  inline double GetTemperature_K(void) const { return temperature_; }
  inline double GetTemperature_deg(void) const { return K2degC(temperature_); }
  inline double GetCapacity(void) const { return capacity_; }
  inline double GetSolarRadiation(void) const { return solar_radiation_; }
  inline NodeType GetNodeType(void) const { return node_type_; }

  // Setter
  inline void SetTemperature_K(double temp_K) { temperature_ = temp_K; }

  // for debug
  void PrintParam(void);
};

#endif  // S2E_DYNAMICS_THERMAL_NODE_HPP_
