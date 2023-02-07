/**
 * @file disturbances.hpp
 * @brief Class to manage all disturbances
 */

#ifndef S2E_DISTURBANCES_DISTURBANCES_H_
#define S2E_DISTURBANCES_DISTURBANCES_H_

#include <vector>

#include "../Environment/Global/SimTime.h"
#include "../simulation/spacecraft/structure/structure.hpp"
#include "SimpleDisturbance.h"
#include "acceleration_disturbance.hpp"

class Logger;

/**
 * @class Disturbances
 * @brief Class to manage all disturbances
 */
class Disturbances {
 public:
  /**
   * @fn Disturbances
   * @brief Constructor
   */
  Disturbances(const SimulationConfig* sim_config, const int sat_id, const Structure* structure, const GlobalEnvironment* glo_env);
  /**
   * @fn ~Disturbances
   * @brief Destructor
   */
  virtual ~Disturbances();

  /**
   * @fn Update
   * @brief Update all disturbance calculation
   */
  void Update(const LocalEnvironment& local_env, const Dynamics& dynamics, const SimTime* sim_time);
  /**
   * @fn LogSetup
   * @brief log setup for all disturbances
   */
  void LogSetup(Logger& logger);

  /**
   * @fn GetTorque
   * @brief Return total disturbance torque in the body frame [Nm]
   */
  Vector<3> GetTorque();
  /**
   * @fn GetTorque
   * @brief Return total disturbance force in the body frame [N]
   */
  Vector<3> GetForce();
  /**
   * @fn GetTorque
   * @brief Return total disturbance acceleration in the inertial frame [m/s2]
   */
  Vector<3> GetAccelerationI();

 private:
  std::string ini_fname_;  //!< Initialization file name

  std::vector<SimpleDisturbance*> disturbances_;       //!< List of disturbances
  Vector<3> sum_torque_;                               //!< Total disturbance torque in the body frame [Nm]
  Vector<3> sum_force_;                                //!< Total disturbance force in the body frame [N]
  vector<AccelerationDisturbance*> acc_disturbances_;  //!< List of acceleration disturbances
  Vector<3> sum_acceleration_i_;                       //!< Total disturbance acceleration in the inertial frame [m/s2]

  /**
   * @fn InitializeInstances
   * @brief Initialize all disturbance class
   */
  void InitializeInstances(const SimulationConfig* sim_config, const int sat_id, const Structure* structure, const GlobalEnvironment* glo_env);
  /**
   * @fn InitializeForceAndTorque
   * @brief Initialize disturbance force and torque
   */
  void InitializeForceAndTorque();
  /**
   * @fn InitializeAcceleration
   * @brief Initialize disturbance acceleration
   */
  void InitializeAcceleration();
};

#endif  // S2E_DISTURBANCES_DISTURBANCES_H_
