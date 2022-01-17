#pragma once

#ifndef __node_H__
#define __node_H__

#include <string>
#include <vector>

#include "../../Interface/LogOutput/Logger.h"

class Node {
protected:
  int node_id_;            // node番号
  std::string node_label_; // node name
  int heater_node_id_;     // heater node番号
  double temperature_;     // 温度[K]
  double capacity_;        // 熱容量[J/K]
  double internal_heat_;   // 内部生成熱[J]
  double alpha_;
  double area_; //太陽熱が入射する面の面積[m^2]
  Vector<3> normal_v_b_; //太陽熱が入射する面の法線ベクトル(機体固定座標系)
  double
      solar_radiation_; //入射する太陽輻射熱[W]([J]に変換するためには時間をかけないといけないことに注意

  double K2deg(double kelvin) const; // 絶対温度からdegCに変換

public:
  Node(const int node_id, const std::string node_label,
       const int heater_node_id, const double temperature_ini,
       const double capacity_ini, const double internal_heat_ini,
       const double alpha, const double area, Vector<3> normal_v_b);
  virtual ~Node();

  // 熱計算用関数
  double CalcSolarRadiation(Vector<3> sun_direction); // 太陽入射熱を計算

  // Output from this class
  int GetNodeId(void) const;
  std::string GetNodeLabel(void) const;
  int GetHeaterNodeId(void) const;
  double GetTemperature_K(void) const;
  double GetTemperature_deg(void) const;
  double GetCapacity(void) const;
  double GetInternalHeat(void) const;
  double GetSolarRadiation(void) const;

  // Setter
  void SetTemperature_K(double temp_K);
  void SetInternalHeat(double heat_power); //内部発熱を計算

  // for debug
  void PrintParam(void);
};
#endif //__node_H__
