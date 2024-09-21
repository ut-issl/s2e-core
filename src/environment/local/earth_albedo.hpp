/**
 * @file earth_albedo.hpp
 * @brief Class to manage earth albedo
 */

#ifndef S2E_ENVIRONMENT_LOCAL_EARTH_ALBEDO_HPP_
#define S2E_ENVIRONMENT_LOCAL_EARTH_ALBEDO_HPP_

#include "environment/global/physical_constants.hpp"
#include "environment/local/local_celestial_information.hpp"

/**
 * @class EarthAlbedo
 * @brief Class to calculate Solar Radiation Pressure
 */
class EarthAlbedo : public ILoggable {
 public:
  bool IsCalcEarthAlbedoEnabled = false;  //!< Calculation flag
  double earth_albedo_factor_ = 0.3;            //!< Earth albedo factor

  /**
   * @fn EarthAlbedo
   * @brief Constructor
   */
  EarthAlbedo();

  /**
   * @fn ~EarthAlbedo
   * @brief Destructor
   */
  virtual ~EarthAlbedo() {}

  // Getter
  /**
   * @fn GetEarthAlbedoFactor
   * @brief Return earth albedo factor
   */
  inline double GetEarthAlbedoFactor() const { return IsCalcEarthAlbedoEnabled ? earth_albedo_factor_ : 0.0; }

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

};

/**
 * @fn InitEarthAlbedo
 * @brief Initialize solar radiation pressure
 * @param [in] initialize_file_path: Path to initialize file
 */
EarthAlbedo InitEarthAlbedo(std::string initialize_file_path);

#endif  // S2E_ENVIRONMENT_LOCAL_EARTH_ALBEDO_HPP_
