/**
 * @file node.hpp
 * @brief thermal calculation node
 */

#ifndef S2E_DYNAMICS_THERMAL_NODE_HPP_
#define S2E_DYNAMICS_THERMAL_NODE_HPP_

#include <library/logger/logger.hpp>
#include <string>
#include <vector>

class Node {
 protected:
  int node_id_;             // node番号
  std::string node_label_;  // node name
  int heater_node_id_;      // heater node番号
  double temperature_;      // 温度[K]
  double capacity_;         // 熱容量[J/K]
  double alpha_rad_;
  double area_;                  // 太陽熱が入射する面の面積[m^2]
  libra::Vector<3> normal_v_b_;  // 太陽熱が入射する面の法線ベクトル(機体固定座標系)
  double solar_radiation_;       // 入射する太陽輻射熱[W]([J]に変換するためには時間をかけないといけないことに注意
  int node_type_;                // ノードの種類 (0: diffusive, 1: boundary, 2: arithmetic)

  double K2deg(double kelvin) const;  // 絶対温度からdegCに変換

 public:
  Node(const int node_id, const std::string node_label, const int node_type, const int heater_node_id, const double temperature_ini,
       const double capacity_ini, const double alpha, const double area, libra::Vector<3> normal_v_b);
  virtual ~Node();

  // 熱計算用関数
  double CalcSolarRadiation(libra::Vector<3> sun_direction);  // 太陽入射熱を計算

  // Output from this class
  inline int Node::GetNodeId(void) const { return node_id_; }
  inline std::string Node::GetNodeLabel(void) const { return node_label_; }
  inline int Node::GetHeaterNodeId(void) const { return heater_node_id_; }
  inline double Node::GetTemperature_K(void) const { return temperature_; }
  inline double Node::GetTemperature_deg(void) const { return K2deg(temperature_); }
  inline double Node::GetCapacity(void) const { return capacity_; }
  inline double Node::GetSolarRadiation(void) const { return solar_radiation_; }
  inline int Node::GetNodeType(void) const { return node_type_; }

  // Setter
  inline void Node::SetTemperature_K(double temp_K) { temperature_ = temp_K; }

  // for debug
  void PrintParam(void);
};

#endif  // S2E_DYNAMICS_THERMAL_NODE_HPP_
