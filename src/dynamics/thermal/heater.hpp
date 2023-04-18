/**
 * @file heater.hpp
 * @brief heater for thermal control
 */

#ifndef S2E_DYNAMICS_THERMAL_HEATER_HPP_
#define S2E_DYNAMICS_THERMAL_HEATER_HPP_

#include <library/logger/logger.hpp>
#include <string>
#include <vector>

class Heater {
 protected:
  int heater_id_;                  // heater番号 (1から始めること)
  std::string heater_label_;       // heater name
  double power_rating_;            // 電力定格[W]
  double lower_threshold_;         // 制御閾値下限[degC]
  double upper_threshold_;         // 制御閾値上限[degC]

  double K2deg(double kelvin) const;  // 絶対温度からdegCに変換
  double deg2K(double degC) const;    // degCから絶対温度に変換

  bool status_;          // ヒーター状態 (1: ON, 0: OFF)
  double power_output_;  // 現在のヒーター出力[W]

 public:
  Heater(const int heater_id, const std::string heater_label, const double power_rating, const double lower_threshold, const double upper_threshold);
  virtual ~Heater();

  // Output from this class
  int GetHeaterId(void) const;
  std::string GetHeaterLabel(void) const;
  double GetPowerRating(void) const;
  double GetLowerThreshold_deg(void) const;
  double GetUpperThreshold_deg(void) const;
  double GetLowerThreshold_K(void) const;
  double GetUpperThreshold_K(void) const;
  double GetStatus(void) const;
  double GetPowerOutput(void) const;

  // Setter
  void SetLowerThreshold(double lower_threshold_deg);
  void SetUpperThreshold(double upper_threshold_deg);
  void SetStatus(bool status);

  // for debug
  void PrintParam(void);
};

#endif  // S2E_DYNAMICS_THERMAL_HEATER_HPP_
