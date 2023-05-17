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

/**
 * @enum HeaterStatus
 * @brief Status of heater (On/Off)
 *
 */
enum class HeaterStatus {
  kOff,
  kOn,
};

/**
 * @class Heater
 * @brief class for heater hardware
 */
class Heater {
 protected:
  unsigned int heater_id_;      // heater id (Use values over 1)
  double power_rating_W_;       // Power Rating (100% Duty) [W]

  HeaterStatus heater_status_;  // Power Status of Heater
  double power_output_W_;       // Power Output of Heater [W]

 public:
  /**
   * @fn Heater
   * @brief Construct a new Heater object
   *
   * @param [in] heater_id
   * @param [in] power_rating_W: Power Rating (100% Duty Output) of Heater [W]
   */
  Heater(const int heater_id, const double power_rating_W);
  /**
   * @fn ~Heater
   * @brief Destroy the Heater object
   */
  virtual ~Heater();

  // Getter
  /**
   * @fn GetHeaterID
   * @brief Return Heater Id
   */
  inline int GetHeaterId(void) const { return heater_id_; }
  /**
   * @fn GetPowerRating_W
   * @brief Return power rating [W]
   */
  inline double GetPowerRating_W(void) const { return power_rating_W_; }
  /**
   * @fn GetHeaterStatus
   * @brief Return HeaterStatus
   */
  inline HeaterStatus GetHeaterStatus(void) const { return heater_status_; }
  /**
   * @fn GetPowerOutput_W
   * @brief Return Power Output of Heater [W]
   */
  inline double GetPowerOutput_W(void) const { return power_output_W_; }

  // Setter
  /**
   * @fn SetHeaterStatus
   * @brief Set the Heater Status
   */
  void SetHeaterStatus(HeaterStatus heater_status);

  // for debug
  /**
   * @fn PrintParam
   * @brief Print power_rating_W, heater_status_, power_output_W_
   */
  void PrintParam(void);
};

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
  void ControlHeater(Heater heater, double temperature_degC);
};

#endif  // S2E_DYNAMICS_THERMAL_HEATER_HPP_
