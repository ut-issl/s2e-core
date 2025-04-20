/**
 * @file local_environment.hpp
 * @brief Class to manage local environments
 */

#ifndef S2E_ENVIRONMENT_LOCAL_LOCAL_ENVIRONMENT_HPP_
#define S2E_ENVIRONMENT_LOCAL_LOCAL_ENVIRONMENT_HPP_

#include "atmosphere.hpp"
#include "dynamics/dynamics.hpp"
#include "earth_albedo.hpp"
#include "earth_infrared.hpp"
#include "environment/global/global_environment.hpp"
#include "geomagnetic_field.hpp"
#include "local_celestial_information.hpp"
#include "simulation/simulation_configuration.hpp"
#include "solar_radiation_pressure_environment.hpp"

namespace s2e::dynamics {
class Dynamics;
}

namespace s2e::environment {

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
  LocalEnvironment(const simulation::SimulationConfiguration* simulation_configuration, const environment::GlobalEnvironment* global_environment,
                   const int spacecraft_id);
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
  void Update(const dynamics::Dynamics* dynamics, const SimulationTime* simulation_time);

  /**
   * @fn LogSetup
   * @brief Log setup for local environments
   */
  void LogSetup(logger::Logger& logger);

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
   * @fn GetEarthAlbedo
   * @brief Return EarthAlbedo class
   */
  inline const EarthAlbedo& GetEarthAlbedo() const { return *earth_albedo_; }
  /**
   * @fn GetEarthInfrared
   * @brief Return EarthInfrared class
   */
  inline const EarthInfrared& GetEarthInfrared() const { return *earth_infrared_; }
  /**
   * @fn GetCelestialInformation
   * @brief Return LocalCelestialInformation class
   */
  inline const LocalCelestialInformation& GetCelestialInformation() const { return *celestial_information_; }

 private:
  Atmosphere* atmosphere_;                                                   //!< Atmospheric density of the earth
  GeomagneticField* geomagnetic_field_;                                      //!< Magnetic field of the earth
  SolarRadiationPressureEnvironment* solar_radiation_pressure_environment_;  //!< Solar radiation pressure
  EarthAlbedo* earth_albedo_;                                                //!< Earth albedo
  EarthInfrared* earth_infrared_;                                            //!< Earth infrared
  LocalCelestialInformation* celestial_information_;                         //!< Celestial information

  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] simulation_configuration: Simulation configuration
   * @param [in] global_environment: Global environment
   * @param [in] spacecraft_id: Satellite ID
   */
  void Initialize(const simulation::SimulationConfiguration* simulation_configuration, const environment::GlobalEnvironment* global_environment,
                  const int spacecraft_id);
};

}  // namespace s2e::environment

#endif  // S2E_ENVIRONMENT_LOCAL_LOCAL_ENVIRONMENT_HPP_
