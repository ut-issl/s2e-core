/**
 * @file global_environment.hpp
 * @brief Class to manage the global environment
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_GLOBAL_ENVIRONMENT_H_
#define S2E_ENVIRONMENT_GLOBAL_GLOBAL_ENVIRONMENT_H_

#include <Interface/LogOutput/Logger.h>

#include <simulation/simulation_configuration.hpp>

#include "GnssSatellites.h"
#include "HipparcosCatalogue.h"
#include "SimTime.h"
#include "celestial_information.hpp"

/**
 * @class GlobalEnvironment
 * @brief Class to manage the global environment
 */
class GlobalEnvironment {
 public:
  /**
   * @fn ~GlobalEnvironment
   * @brief Constructor
   * @param [in] sim_config: Simulation configuration
   */
  GlobalEnvironment(SimulationConfig* sim_config);
  /**
   * @fn ~GlobalEnvironment
   * @brief Destructor
   */
  ~GlobalEnvironment();

  /**
   * @fn Initialize
   * @brief Initialize all global environment members
   * @param [in] sim_config: Simulation configuration
   */
  void Initialize(SimulationConfig* sim_config);
  /**
   * @fn Update
   * @brief Update states of all global environment
   */
  void Update();
  /**
   * @fn LogSetup
   * @brief Log setup of global environment information
   */
  void LogSetup(Logger& logger);
  /**
   * @fn Reset
   * @brief Reset clock of SimTime
   */
  void Reset(void);

  // Getter
  /**
   * @fn GetSimTime
   * @brief Return SimTime
   */
  inline const SimTime& GetSimTime() const { return *sim_time_; }
  /**
   * @fn GetCelesInfo
   * @brief Return CelestialInformation
   */
  inline const CelestialInformation& GetCelesInfo() const { return *celes_info_; }
  /**
   * @fn GetHippCatalog
   * @brief Return HipparcosCatalogue
   */
  inline const HipparcosCatalogue& GetHippCatalog() const { return *hipp_; }
  /**
   * @fn GetGnssSatellites
   * @brief Return GnssSatellites
   */
  inline const GnssSatellites& GetGnssSatellites() const { return *gnss_satellites_; }

 private:
  SimTime* sim_time_;                 //!< Simulation time
  CelestialInformation* celes_info_;  //!< Celestial bodies information
  HipparcosCatalogue* hipp_;          //!< Hipparcos catalogue
  GnssSatellites* gnss_satellites_;   //!< GNSS satellites
};

#endif  // S2E_ENVIRONMENT_GLOBAL_GLOBAL_ENVIRONMENT_H_
