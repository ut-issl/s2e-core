/**
 * @file heater.hpp
 * @brief heater object for thermal control
 */

#ifndef S2E_DYNAMICS_THERMAL_HEATER_HPP_
#define S2E_DYNAMICS_THERMAL_HEATER_HPP_

#include <environment/global/physical_constants.hpp>
#include <logger/logger.hpp>
#include <string>
#include <vector>

namespace s2e::dynamics::thermal {

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
 public:
  /**
   * @fn Heater
   * @brief Construct a new Heater object
   *
   * @param [in] heater_id
   * @param [in] power_rating_W: Power Rating (100% Duty Output) of Heater [W]
   */
  Heater(const size_t heater_id, const double power_rating_W);
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
  inline size_t GetHeaterId(void) const { return heater_id_; }
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

 protected:
  size_t heater_id_;       //!< heater id (Use values over 1)
  double power_rating_W_;  //!</ Power Rating (100% Duty) [W]

  HeaterStatus heater_status_;  //!< Power Status of Heater
  double power_output_W_;       //!< Power Output of Heater [W]

  /**
   * @fn AssertHeaterParams
   * @brief Check Heater Parameters
   */
  void AssertHeaterParams(void);
};

/**
 * @fn InitHeater
 * @brief Initialize Heater object from csv file
 * @param[in] heater_str: str read from csv file
 * @return Heater
 */
Heater InitHeater(const std::vector<std::string>& heater_str);

}  // namespace s2e::dynamics::thermal

#endif  // S2E_DYNAMICS_THERMAL_HEATER_HPP_
