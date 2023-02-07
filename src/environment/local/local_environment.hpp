/**
 * @file local_environment.hpp
 * @brief Class to manage local environments
 */

#ifndef S2E_ENVIRONMENT_LOCAL_LOCAL_ENVIRONMENT_H_
#define S2E_ENVIRONMENT_LOCAL_LOCAL_ENVIRONMENT_H_

#include <dynamics/Dynamics.h>

#include <environment/global/global_environment.hpp>

#include "atmosphere.hpp"
#include "geomagnetic_field.hpp"
#include "local_celestial_information.hpp"
#include "simulation/simulation_configuration.hpp"
#include "solar_radiation_pressure_environment.hpp"

class Logger;
class SimTime;

/**
 * @class LocalEnvironment
 * @brief Class to manage local environments
 */
class LocalEnvironment {
 public:
  /**
   * @fn LocalEnvironment
   * @brief Constructor
   * @param [in] sim_config: Simulation configuration
   * @param [in] glo_env: Global environment
   * @param [in] sat_id: Satellite ID
   */
  LocalEnvironment(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);
  /**
   * @fn ~LocalEnvironment
   * @brief Destructor
   */
  ~LocalEnvironment();
  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] sim_config: Simulation configuration
   * @param [in] glo_env: Global environment
   * @param [in] sat_id: Satellite ID
   */
  void Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);

  /**
   * @fn Update
   * @brief Update all states
   * @param [in] dynamics: Dynamics information of the satellite
   * @param [in] sim_time: Simulation time
   */
  void Update(const Dynamics* dynamics, const SimTime* sim_time);

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
   * @fn GetMag
   * @brief Return MagEnvironment class
   */
  inline const MagEnvironment& GetMag() const { return *mag_; }
  /**
   * @fn GetSrp
   * @brief Return SRPEnvironment class
   */
  inline const SRPEnvironment& GetSrp() const { return *srp_; }
  /**
   * @fn GetCelesInfo
   * @brief Return LocalCelestialInformation class
   */
  inline const LocalCelestialInformation& GetCelesInfo() const { return *celes_info_; }

 private:
  Atmosphere* atmosphere_;                 //!< Atmospheric density of the earth
  MagEnvironment* mag_;                    //!< Magnetic field of the earth
  SRPEnvironment* srp_;                    //!< Solar radiation pressure
  LocalCelestialInformation* celes_info_;  //!< Celestial information
};

#endif  // S2E_ENVIRONMENT_LOCAL_LOCAL_ENVIRONMENT_H_
