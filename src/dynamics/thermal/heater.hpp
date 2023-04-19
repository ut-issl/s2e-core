/**
 * @file heater.hpp
 * @brief heater for thermal control
 */

#ifndef S2E_DYNAMICS_THERMAL_HEATER_HPP_
#define S2E_DYNAMICS_THERMAL_HEATER_HPP_

#include <environment/global/physical_constants.hpp>
#include <library/logger/logger.hpp>
#include <string>
#include <vector>

enum class HeaterStatus {
  kOff,
  kOn,
};

class Heater {
 protected:
  unsigned int heater_id_;    // heater id (Use values over 1)
  std::string heater_label_;  // heater name
  double power_rating_;       // Power Rating (100% Duty) [W]
  double lower_threshold_;    // Lower Threshold of Heater Control [degC]
  double upper_threshold_;    // Upper Threshold of Heater Control [degC]

  HeaterStatus heater_status_;  // Power Status of Heater
  double power_output_;         // Power Output of Heater [W]

 public:
  Heater(const int heater_id, const std::string heater_label, const double power_rating, const double lower_threshold, const double upper_threshold);
  virtual ~Heater();

  // Output from this class
  inline int GetHeaterId(void) const { return heater_id_; }
  inline std::string GetHeaterLabel(void) const { return std::string(); }
  inline double GetPowerRating(void) const { return power_rating_; }
  inline double GetLowerThreshold_deg(void) const { return lower_threshold_; }
  inline double GetUpperThreshold_deg(void) const { return upper_threshold_; }
  inline double GetLowerThreshold_K(void) const { return degC2K(lower_threshold_); }
  inline double GetUpperThreshold_K(void) const { return degC2K(upper_threshold_); }
  inline HeaterStatus GetHeaterStatus(void) const { return heater_status_; }
  inline double GetPowerOutput(void) const { return power_output_; }

  // Setter
  inline void SetLowerThreshold(double lower_threshold_deg) { lower_threshold_ = lower_threshold_deg; }
  inline void SetUpperThreshold(double upper_threshold_deg) { upper_threshold_ = upper_threshold_deg; }
  inline void SetHeaterStatus(HeaterStatus heater_status) { heater_status_ = heater_status; };

  // for debug
  void PrintParam(void);
};

#endif  // S2E_DYNAMICS_THERMAL_HEATER_HPP_
