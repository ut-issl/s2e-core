/**
 * @file spacecraft.hpp
 * @brief Definition of Spacecraft class
 */

#ifndef S2E_SIMULATION_SPACECRAFT_SPACECRAFT_H_
#define S2E_SIMULATION_SPACECRAFT_SPACECRAFT_H_

#include <Disturbance/Disturbances.h>
#include <Dynamics/Dynamics.h>
#include <Environment/Global/ClockGenerator.h>
#include <Environment/Local/LocalEnvironment.h>
#include <RelativeInformation/RelativeInformation.h>

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
  Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);
  /**
   * @fn Spacecraft
   * @brief Constructor for multiple satellite simulation
   */
  Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* rel_info, const int sat_id);

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
  virtual void Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);
  /**
   * @fn Initialize
   * @brief Initialize function for multiple spacecraft simulation
   */
  virtual void Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* rel_info, const int sat_id);

  /**
   * @fn Update
   * @brief Update all states related with the spacecraft
   */
  virtual void Update(const SimTime* sim_time);

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
   * @fn GetlocalEnv
   * @brief Get local environment around the spacecraft
   */
  inline const LocalEnvironment& GetLocalEnv() const { return *local_env_; }
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
   * @fn GetSatID
   * @brief Get ID of the spacecraft
   */
  inline int GetSatID() const { return sat_id_; }

 protected:
  ClockGenerator clock_gen_;         //!< Origin of clock for the spacecraft
  Dynamics* dynamics_;               //!< Dynamics information of the spacecraft
  RelativeInformation* rel_info_;    //!< Relative information with respect to the other spacecraft
  LocalEnvironment* local_env_;      //!< Local environment information around the spacecraft
  Disturbances* disturbances_;       //!< Disturbance information acting on the spacecraft
  Structure* structure_;             //!< Structure information of the spacecraft
  InstalledComponents* components_;  //!< Components information installed on the spacecraft
  const int sat_id_;                 //!< ID of the spacecraft
};

#endif  // S2E_SIMULATION_SPACECRAFT_SPACECRAFT_H_
