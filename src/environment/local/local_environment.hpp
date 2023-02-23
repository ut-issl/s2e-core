/**
 * @file local_environment.hpp
 * @brief Class to manage local environments
 */

#ifndef S2E_ENVIRONMENT_LOCAL_LOCAL_ENVIRONMENT_HPP_
#define S2E_ENVIRONMENT_LOCAL_LOCAL_ENVIRONMENT_HPP_

#include "atmosphere.hpp"
#include "dynamics/dynamics.hpp"
#include "environment/global/global_environment.hpp"
#include "geomagnetic_field.hpp"
#include "local_celestial_information.hpp"
#include "simulation/simulation_configuration.hpp"
#include "solar_radiation_pressure_environment.hpp"

/**
 * @class LocalEnvironment
 * @brief Class to manage local environments
 */
class LocalEnvironment {
 public:
  /**
   * @fn LocalEnvironment
   * @brief Constructor
   * @param [in] simulation_configuration: Simulation configuration
   * @param [in] global_environment: Global environment
   * @param [in] spacecraft_id: Satellite ID
   */
  LocalEnvironment(const SimulationConfig* simulation_configuration, const GlobalEnvironment* global_environment, const int spacecraft_id);
  /**
   * @fn ~LocalEnvironment
   * @brief Destructor
   */
  ~LocalEnvironment();

  /**
   * @fn Update
   * @brief Update all states
   * @param [in] dynamics: Dynamics information of the satellite
   * @param [in] simulation_time: Simulation time
   */
  void Update(const Dynamics* dynamics, const SimTime* simulation_time);

  /**
   * @fn LogSetup
   * @brief Log setup for local environments
   */
  void LogSetup(Logger& logger);

  /**
   * @fn GetAtmosphere
   * @brief Return Atmosphere class
   */
  inline const Atmosphere& GetAtmosphere() const { return *atmosphere_; }
  /**
   * @fn GetGeomagneticField
   * @brief Return GeomagneticField class
   */
  inline const GeomagneticField& GetGeomagneticField() const { return *geomagnetic_field_; }
  /**
   * @fn GetSolarRadiationPressure
   * @brief Return SolarRadiationPressureEnvironment class
   */
  inline const SolarRadiationPressureEnvironment& GetSolarRadiationPressure() const { return *solar_radiation_pressure_environment_; }
  /**
   * @fn GetCelestialInformation
   * @brief Return LocalCelestialInformation class
   */
  inline const LocalCelestialInformation& GetCelestialInformation() const { return *celestial_information_; }

 private:
  Atmosphere* atmosphere_;                                                   //!< Atmospheric density of the earth
  GeomagneticField* geomagnetic_field_;                                      //!< Magnetic field of the earth
  SolarRadiationPressureEnvironment* solar_radiation_pressure_environment_;  //!< Solar radiation pressure
  LocalCelestialInformation* celestial_information_;                         //!< Celestial information

  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] simulation_configuration: Simulation configuration
   * @param [in] global_environment: Global environment
   * @param [in] spacecraft_id: Satellite ID
   */
  void Initialize(const SimulationConfig* simulation_configuration, const GlobalEnvironment* global_environment, const int spacecraft_id);
};

#endif  // S2E_ENVIRONMENT_LOCAL_LOCAL_ENVIRONMENT_HPP_
