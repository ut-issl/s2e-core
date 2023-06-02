/**
 * @file heater_controller.hpp
 * @brief heater controller
 */

#ifndef S2E_DYNAMICS_THERMAL_HEATER_CONTROLLER_HPP_
#define S2E_DYNAMICS_THERMAL_HEATER_CONTROLLER_HPP_

#include <environment/global/physical_constants.hpp>
#include <library/logger/logger.hpp>
#include <string>
#include <vector>
#include "heater.hpp"

class HeaterController {
 protected:
  double lower_threshold_degC_;  // Lower Threshold of Heater Control [degC]
  double upper_threshold_degC_;  // Upper Threshold of Heater Control [degC]

 public:
  /**
   * @fn HeaterController
   * @brief Construct a new Heater Controller object
   * @param[in] lower_threshold_degC: Lower threshold of heater control [degC]
   * @param[in] upper_threshold_degC: Upper threshold of heater control [degC]
   */
  HeaterController(const double lower_threshold_degC, const double upper_threshold_degC);
  /**
   * @fn ~HeaterController
   * @brief Destroy the Heater Controller object
   */
  virtual ~HeaterController();

  // Getter
  /**
   * @fn GetLowerThreshold_degC
   * @brief Return Lower Thershold [degC]
   */
  inline double GetLowerThreshold_degC(void) const { return lower_threshold_degC_; }
  /**
   * @fn GetUpperThreshold_degC
   * @brief Return Upper Thershold [degC]
   */
  inline double GetUpperThreshold_degC(void) const { return upper_threshold_degC_; }
  /**
   * @fn GetLowerThreshold_K
   * @brief Return Lower Thershold [K]
   */
  inline double GetLowerThreshold_K(void) const { return degC2K(lower_threshold_degC_); }
  /**
   * @fn GetUpperThreshold_K
   * @brief Return Upper Thershold [K]
   */
  inline double GetUpperThreshold_K(void) const { return degC2K(upper_threshold_degC_); }

  // Setter
  /**
   * @brief Set the Lower Threshold [degC]
   * @param[in] lower_threshold_degC
   */
  inline void SetLowerThreshold(double lower_threshold_degC) { lower_threshold_degC_ = lower_threshold_degC; }
  /**
   * @brief Set the Upper Threshold [degC]
   * @param[in] upper_threshold_degC
   */
  inline void SetUpperThreshold(double upper_threshold_degC) { upper_threshold_degC_ = upper_threshold_degC; }

  /**
   * @fn ControlHeater
   * @brief Compare temperature_degC and lower_threshold_degC / upper_threshold_degC and switch HeaterStatus of heater depending on comparison results
   * @param[in] heater
   * @param[in] temperature_degC
   */
  void ControlHeater(Heater* p_heater, double temperature_degC);
};

#endif  // S2E_DYNAMICS_THERMAL_HEATER_CONTROLLER_HPP_
