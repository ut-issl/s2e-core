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
  unsigned int heater_id_;  // heater id (Use values over 1)
  double power_rating_;     // Power Rating (100% Duty) [W]

  HeaterStatus heater_status_;  // Power Status of Heater
  double power_output_;         // Power Output of Heater [W]

 public:
  Heater(const int heater_id, const double power_rating);
  virtual ~Heater();

  // Output from this class
  inline int GetHeaterId(void) const { return heater_id_; }
  inline double GetPowerRating(void) const { return power_rating_; }

  inline HeaterStatus GetHeaterStatus(void) const { return heater_status_; }
  inline double GetPowerOutput(void) const { return power_output_; }

  // Setter
  void SetHeaterStatus(HeaterStatus heater_status);

  // for debug
  void PrintParam(void);
};

class HeaterController {
 protected:
  double lower_threshold_;  // Lower Threshold of Heater Control [degC]
  double upper_threshold_;  // Upper Threshold of Heater Control [degC]

 public:
  HeaterController(const double lower_threshold, const double upper_threshold);
  virtual ~HeaterController();

  inline double GetLowerThreshold_degC(void) const { return lower_threshold_; }
  inline double GetUpperThreshold_degC(void) const { return upper_threshold_; }
  inline double GetLowerThreshold_K(void) const { return degC2K(lower_threshold_); }
  inline double GetUpperThreshold_K(void) const { return degC2K(upper_threshold_); }

  inline void SetLowerThreshold(double lower_threshold_degC) { lower_threshold_ = lower_threshold_degC; }
  inline void SetUpperThreshold(double upper_threshold_degC) { upper_threshold_ = upper_threshold_degC; }

  void ControlHeater(Heater heater, double temperature_degC);
};

#endif  // S2E_DYNAMICS_THERMAL_HEATER_HPP_
