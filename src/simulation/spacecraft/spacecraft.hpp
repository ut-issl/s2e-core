/**
 * @file spacecraft.hpp
 * @brief Definition of Spacecraft class
 */

#ifndef S2E_SIMULATION_SPACECRAFT_SPACECRAFT_HPP_
#define S2E_SIMULATION_SPACECRAFT_SPACECRAFT_HPP_

#include <disturbances/disturbances.hpp>
#include <dynamics/dynamics.hpp>
#include <environment/global/clock_generator.hpp>
#include <environment/local/local_environment.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

#include "installed_components.hpp"
#include "structure/structure.hpp"

/**
 * @class Spacecraft
 * @brief Base class to express Spacecraft
 */
class Spacecraft {
 public:
  /**
   * @fn Spacecraft
   * @brief Constructor for single satellite simulation
   */
  Spacecraft(const SimulationConfig* simulation_configuration, const GlobalEnvironment* global_environment, const int spacecraft_id);
  /**
   * @fn Spacecraft
   * @brief Constructor for multiple satellite simulation
   */
  Spacecraft(const SimulationConfig* simulation_configuration, const GlobalEnvironment* global_environment, RelativeInformation* relative_information,
             const int spacecraft_id);

  /**
   * @fn ~Spacecraft
   * @brief Destructor
   */
  virtual ~Spacecraft();

  // forbidden copy
  Spacecraft(const Spacecraft&) = delete;
  Spacecraft& operator=(const Spacecraft&) = delete;

  // Virtual functions
  /**
   * @fn Initialize
   * @brief Initialize function for single spacecraft simulation
   */
  virtual void Initialize(const SimulationConfig* simulation_configuration, const GlobalEnvironment* global_environment, const int spacecraft_id);
  /**
   * @fn Initialize
   * @brief Initialize function for multiple spacecraft simulation
   */
  virtual void Initialize(const SimulationConfig* simulation_configuration, const GlobalEnvironment* global_environment,
                          RelativeInformation* relative_information, const int spacecraft_id);

  /**
   * @fn Update
   * @brief Update all states related with the spacecraft
   */
  virtual void Update(const SimulationTime* simulation_time);

  /**
   * @fn Clear
   * @brief Clear force and torque acting on the spacecraft
   */
  virtual void Clear(void);

  /**
   * @fn LogSetup
   * @brief Logger setting for the spacecraft specific information
   */
  virtual void LogSetup(Logger& logger);

  // Getters
  /**
   * @fn GetDynamics
   * @brief Get dynamics of the spacecraft
   */
  inline const Dynamics& GetDynamics() const { return *dynamics_; }
  /**
   * @fn GetLocalEnvironment
   * @brief Get local environment around the spacecraft
   */
  inline const LocalEnvironment& GetLocalEnvironment() const { return *local_environment_; }
  /**
   * @fn GetDisturbances
   * @brief Get disturbance acting of the spacecraft
   */
  inline const Disturbances& GetDisturbances() const { return *disturbances_; }
  /**
   * @fn GetStructure
   * @brief Get structure of the spacecraft
   */
  inline const Structure& GetStructure() const { return *structure_; }
  /**
   * @fn GetInstalledComponents
   * @brief Get components installed on the spacecraft
   */
  inline const InstalledComponents& GetInstalledComponents() const { return *components_; }
  /**
   * @fn GetSpacecraftId
   * @brief Get ID of the spacecraft
   */
  inline unsigned int GetSpacecraftId() const { return spacecraft_id_; }

 protected:
  ClockGenerator clock_generator_;             //!< Origin of clock for the spacecraft
  Dynamics* dynamics_;                         //!< Dynamics information of the spacecraft
  RelativeInformation* relative_information_;  //!< Relative information with respect to the other spacecraft
  LocalEnvironment* local_environment_;        //!< Local environment information around the spacecraft
  Disturbances* disturbances_;                 //!< Disturbance information acting on the spacecraft
  Structure* structure_;                       //!< Structure information of the spacecraft
  InstalledComponents* components_;            //!< Components information installed on the spacecraft
  const unsigned int spacecraft_id_;           //!< ID of the spacecraft
};

#endif  // S2E_SIMULATION_SPACECRAFT_SPACECRAFT_HPP_
