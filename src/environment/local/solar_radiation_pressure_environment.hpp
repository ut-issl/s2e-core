/**
 * @file solar_radiation_pressure_environment.hpp
 * @brief Class to calculate Solar Radiation Pressure
 */

#ifndef S2E_ENVIRONMENT_LOCAL_SOLAR_RADIATION_PRESSURE_ENVIRONMENT_HPP_
#define S2E_ENVIRONMENT_LOCAL_SOLAR_RADIATION_PRESSURE_ENVIRONMENT_HPP_

#include "environment/global/physical_constants.hpp"
#include "environment/local/local_celestial_information.hpp"

/**
 * @class SolarRadiationPressureEnvironment
 * @brief Class to calculate Solar Radiation Pressure
 */
class SolarRadiationPressureEnvironment : public ILoggable {
 public:
  bool IsCalcEnabled = true;  //!< Calculation flag

  /**
   * @fn SolarRadiationPressureEnvironment
   * @brief Constructor
   * @param [in] local_celestial_information: Local celestial information
   */
  SolarRadiationPressureEnvironment(LocalCelestialInformation* local_celestial_information);

  /**
   * @fn ~SolarRadiationPressureEnvironment
   * @brief Destructor
   */
  virtual ~SolarRadiationPressureEnvironment() {}

  /**
   * @fn UpdateAllStates
   * @brief Update pressure and shadow coefficients
   */
  void UpdateAllStates();

  // Getter
  /**
   * @fn GetPressure_N_m2
   * @brief Calculate and return solar radiation pressure that takes into account eclipse [N/m^2]
   */
  inline double GetPressure_N_m2() const { return solar_radiation_pressure_N_m2_ * shadow_coefficient_; }
  /**
   * @fn GetPowerDensity_W_m2
   * @brief Calculate and return solar power per unit area considering eclipse [W/m^2]
   */
  inline double GetPowerDensity_W_m2() const { return solar_radiation_pressure_N_m2_ * environment::speed_of_light_m_s * shadow_coefficient_; }
  /**
   * @fn GetPressureWithoutEclipse_Nm2
   * @brief Return solar pressure without eclipse effect [N/m^2]
   */
  inline double GetPressureWithoutEclipse_Nm2() const { return solar_radiation_pressure_N_m2_; }
  /**
   * @fn GetSolarConstant_W_m2
   * @brief Return solar constant value [W/m^2]
   */
  inline double GetSolarConstant_W_m2() const { return solar_constant_W_m2_; }
  /**
   * @fn GetShadowCoefficient
   * @brief Return shadow function
   */
  inline double GetShadowCoefficient() const { return shadow_coefficient_; }
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
  double solar_radiation_pressure_N_m2_;  //!< Solar radiation pressure [N/m^2]
  double solar_constant_W_m2_ = 1366.0;   //!< Solar constant [W/m^2] TODO: We need to change the value depends on sun activity.
  double shadow_coefficient_ = 1.0;       //!< Shadow function
  double sun_radius_m_;                   //!< Sun radius [m]
  std::string shadow_source_name_;        //!< Shadow source name

  LocalCelestialInformation* local_celestial_information_;  //!< Local celestial information

  /**
   * @fn UpdatePressure
   * @brief Update pressure with solar distance
   */
  void UpdatePressure();

  /**
   * @fn CalcShadowCoefficient
   * @brief Calculate shadow coefficient
   * @param [in] shadow_source_name_: Shadow source name
   */
  void CalcShadowCoefficient(std::string shadow_source_name);
};

/**
 * @fn InitSolarRadiationPressureEnvironment
 * @brief Initialize solar radiation pressure
 * @param [in] initialize_file_path: Path to initialize file
 * @param [in] local_celestial_information: Local celestial information
 */
SolarRadiationPressureEnvironment InitSolarRadiationPressureEnvironment(std::string initialize_file_path,
                                                                        LocalCelestialInformation* local_celestial_information);

#endif  // S2E_ENVIRONMENT_LOCAL_SOLAR_RADIATION_PRESSURE_ENVIRONMENT_HPP_
