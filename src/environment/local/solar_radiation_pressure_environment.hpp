/**
 * @file solar_radiation_pressure_environment.hpp
 * @brief Class to calculate Solar Radiation Pressure
 */

#ifndef S2E_ENVIRONMENT_LOCAL_SOLAR_RADIATION_PRESSURE_ENVIRONMENT_H_
#define S2E_ENVIRONMENT_LOCAL_SOLAR_RADIATION_PRESSURE_ENVIRONMENT_H_

#include <interface/log_output/loggable.hpp>

#include <Library/math/Vector.hpp>
#include <environment/local/local_celestial_information.hpp>

using libra::Vector;

/**
 * @class SRPEnvironment
 * @brief Class to calculate Solar Radiation Pressure
 */
class SRPEnvironment : public ILoggable {
 public:
  bool IsCalcEnabled = true;  //!< Calculation flag

  /**
   * @fn SRPEnvironment
   * @brief Constructor
   * @param [in] local_celes_info: Local celestial information
   */
  SRPEnvironment(LocalCelestialInformation* local_celes_info);
  /**
   * @fn ~SRPEnvironment
   * @brief Destructor
   */
  virtual ~SRPEnvironment() {}

  /**
   * @fn UpdateAllStates
   * @brief Update pressure and shadow coefficients
   */
  void UpdateAllStates();
  /**
   * @fn UpdatePressure
   * @brief Update pressure with solar distance
   */
  void UpdatePressure();

  /**
   * @fn CalcTruePressure
   * @brief Calculate and return solar radiation pressure that takes into account eclipse [N/m^2]
   */
  double CalcTruePressure() const;
  /**
   * @fn CalcPowerDensity
   * @brief Calculate and return solar power per unit area considering eclipse [W/m^2]
   */
  double CalcPowerDensity() const;
  /**
   * @fn GetPressure
   * @brief Return solar pressure without eclipse effect [N/m^2]
   */
  double GetPressure() const;
  /**
   * @fn GetSolarConstant
   * @brief Return solar constant value [W/m^2]
   */
  double GetSolarConstant() const;
  /**
   * @fn GetShadowCoefficient
   * @brief Return shadow function
   */
  double GetShadowCoefficient() const;
  /**
   * @fn GetIsEclipsed
   * @brief Returns true if the shadow function is less than 1
   */
  inline bool GetIsEclipsed() const { return (shadow_coefficient_ >= 1.0 ? false : true); }

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

 private:
  double pressure_;                  //!< Solar radiation pressure [N/m^2]
  double solar_constant_;            //!< solar constant [W/m^2] TODO: We need to change the value depends on sun activity.
  double shadow_coefficient_ = 1.0;  //!< shadow function
  double sun_radius_m_;              //!< Sun radius [m]
  std::string shadow_source_name_;   //!< Shadow source name

  LocalCelestialInformation* local_celes_info_;  //!< Local celestial information

  /**
   * @fn CalcShadowCoefficient
   * @brief Calculate shadow coefficient
   * @param [in] shadow_source_name_: Shadow source name
   */
  void CalcShadowCoefficient(std::string shadow_source_name);
};

#endif  // S2E_ENVIRONMENT_LOCAL_SOLAR_RADIATION_PRESSURE_ENVIRONMENT_H_
