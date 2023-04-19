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
  unsigned int heater_id_;    // heater id (Use values over 1)
  std::string heater_label_;  // heater name
  double power_rating_;       // Power Rating (100% Duty) [W]
  double lower_threshold_;    // Lower Threshold of Heater Control [degC]
  double upper_threshold_;    // Upper Threshold of Heater Control [degC]

  double K2deg(double kelvin) const;  // Convert Kelvin to degC
  double deg2K(double degC) const;    // Convert degC to Kelvin

  bool status_;          // Power Status of Heater (True: ON, False: OFF)
  double power_output_;  // Power Output of Heater [W]

 public:
  Heater(const int heater_id, const std::string heater_label, const double power_rating, const double lower_threshold, const double upper_threshold);
  virtual ~Heater();

  // Output from this class
  inline int Heater::GetHeaterId(void) const { return heater_id_; }
  inline std::string Heater::GetHeaterLabel(void) const { return std::string(); }
  inline double Heater::GetPowerRating(void) const { return power_rating_; }
  inline double Heater::GetLowerThreshold_deg(void) const { return lower_threshold_; }
  inline double Heater::GetUpperThreshold_deg(void) const { return upper_threshold_; }
  inline double Heater::GetLowerThreshold_K(void) const { return deg2K(lower_threshold_); }
  inline double Heater::GetUpperThreshold_K(void) const { return deg2K(upper_threshold_); }
  inline double Heater::GetStatus(void) const { return status_; }
  inline double Heater::GetPowerOutput(void) const { return power_output_; }

  // Setter
  inline void Heater::SetLowerThreshold(double lower_threshold_deg) { lower_threshold_ = lower_threshold_deg; }
  inline void Heater::SetUpperThreshold(double upper_threshold_deg) { upper_threshold_ = upper_threshold_deg; }
  void SetStatus(bool status);

  // for debug
  void PrintParam(void);
};

#endif  // S2E_DYNAMICS_THERMAL_HEATER_HPP_
