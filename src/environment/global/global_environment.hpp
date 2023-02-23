/**
 * @file global_environment.hpp
 * @brief Class to manage the global environment
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_GLOBAL_ENVIRONMENT_HPP_
#define S2E_ENVIRONMENT_GLOBAL_GLOBAL_ENVIRONMENT_HPP_

#include "celestial_information.hpp"
#include "gnss_satellites.hpp"
#include "hipparcos_catalogue.hpp"
#include "library/logger/logger.hpp"
#include "simulation/simulation_configuration.hpp"
#include "simulation_time.hpp"

/**
 * @class GlobalEnvironment
 * @brief Class to manage the global environment
 */
class GlobalEnvironment {
 public:
  /**
   * @fn ~GlobalEnvironment
   * @brief Constructor
   * @param [in] simulation_configuration: Simulation configuration
   */
  GlobalEnvironment(SimulationConfig* simulation_configuration);
  /**
   * @fn ~GlobalEnvironment
   * @brief Destructor
   */
  ~GlobalEnvironment();

  /**
   * @fn Initialize
   * @brief Initialize all global environment members
   * @param [in] simulation_configuration: Simulation configuration
   */
  void Initialize(SimulationConfig* simulation_configuration);
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
  inline const SimTime& GetSimulationTime() const { return *simulation_time_; }
  /**
   * @fn GetCelestialInformation
   * @brief Return CelestialInformation
   */
  inline const CelestialInformation& GetCelestialInformation() const { return *celestial_information_; }
  /**
   * @fn GetHippCatalog
   * @brief Return HipparcosCatalogue
   */
  inline const HipparcosCatalogue& GetHipparcosCatalog() const { return *hipparcos_catalogue_; }
  /**
   * @fn GetGnssSatellites
   * @brief Return GnssSatellites
   */
  inline const GnssSatellites& GetGnssSatellites() const { return *gnss_satellites_; }

 private:
  SimTime* simulation_time_;                     //!< Simulation time
  CelestialInformation* celestial_information_;  //!< Celestial bodies information
  HipparcosCatalogue* hipparcos_catalogue_;      //!< Hipparcos catalogue
  GnssSatellites* gnss_satellites_;              //!< GNSS satellites
};

#endif  // S2E_ENVIRONMENT_GLOBAL_GLOBAL_ENVIRONMENT_HPP_
