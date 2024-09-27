/**
 * @file earth_albedo.hpp
 * @brief Class to manage earth albedo
 */

#ifndef S2E_ENVIRONMENT_LOCAL_EARTH_ALBEDO_HPP_
#define S2E_ENVIRONMENT_LOCAL_EARTH_ALBEDO_HPP_

#include "environment/global/physical_constants.hpp"
#include "environment/local/local_celestial_information.hpp"
#include "solar_radiation_pressure_environment.hpp"

namespace s2e::environment {

/**
 * @class EarthAlbedo
 * @brief Class to calculate Solar Radiation Pressure
 */
class EarthAlbedo : public ILoggable {
 public:
  /**
   * @fn EarthAlbedo
   * @brief Constructor
   * @param [in] local_celestial_information: Local celestial information
   */
  EarthAlbedo(LocalCelestialInformation* local_celestial_information, SolarRadiationPressureEnvironment* srp_environment);

  /**
   * @fn ~EarthAlbedo
   * @brief Destructor
   */
  virtual ~EarthAlbedo() {}

  /**
   * @fn UpdateAllStates
   * @brief Update earth albedo
   */
  void UpdateAllStates();

  // Getter
  /**
   * @fn GetEarthAlbedoFactor
   * @brief Return earth albedo factor
   */
  inline double GetEarthAlbedoFactor() const { return earth_albedo_factor_; }
  /**
   * @fn GetPowerDensity_W_m2
   * @brief Calculate and return earth albedo [W/m^2]
   */
  inline double GetEarthAlbedoRadiationPower_W_m2() const { return earth_albedo_W_m2_; }
  /**
   * @fn GetIsEclipsed
   * @brief Returns true if the shadow function is less than 1
   */
  inline bool GetIsCalcEarthAlbedoEnabled() const { return is_calc_earth_albedo_enabled_; }

  // Setter
  /**
   * @fn SetEarthAlbedoFactor
   * @brief Set earth albedo factor
   * @param [in] earth_albedo_factor: Earth albedo factor
   */
  inline void SetEarthAlbedoFactor(const double earth_albedo_factor) { earth_albedo_factor_ = earth_albedo_factor; }
  /**
   * @fn SetIsCalcEarthAlbedoEnabled
   * @brief Set calculation flag
   * @param [in] is_calc_earth_albedo_enabled: Calculation flag
   */
  inline void SetIsCalcEarthAlbedoEnabled(const bool is_calc_earth_albedo_enabled) { is_calc_earth_albedo_enabled_ = is_calc_earth_albedo_enabled; }

 private:
  double earth_albedo_W_m2_ = 0.0;             //!< Earth albedo [W/m^2]
  bool is_calc_earth_albedo_enabled_ = false;  //!< Calculation flag
  double earth_albedo_factor_ = 0.3;           //!< Earth albedo factor

  LocalCelestialInformation* local_celestial_information_;  //!< Local celestial information
  SolarRadiationPressureEnvironment* srp_environment_;      //!< Solar radiation pressure environment

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

  /**
   * @fn CalcEarthAlbedo
   * @brief Calculate earth albedo
   * @param [in] local_celestial_information: Local celestial information
   */
  void CalcEarthAlbedo(const LocalCelestialInformation* local_celestial_information);
};

/**
 * @fn InitEarthAlbedo
 * @brief Initialize solar radiation pressure
 * @param [in] initialize_file_path: Path to initialize file
 */
EarthAlbedo InitEarthAlbedo(std::string initialize_file_path, LocalCelestialInformation* local_celestial_information,
                            SolarRadiationPressureEnvironment* srp_environment);

} // namespace s2e::environment

#endif  // S2E_ENVIRONMENT_LOCAL_EARTH_ALBEDO_HPP_
