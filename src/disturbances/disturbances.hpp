/**
 * @file disturbances.hpp
 * @brief Class to manage all disturbances
 */

#ifndef S2E_DISTURBANCES_DISTURBANCES_HPP_
#define S2E_DISTURBANCES_DISTURBANCES_HPP_

#include <vector>

#include "../environment/global/simulation_time.hpp"
#include "../simulation/spacecraft/structure/structure.hpp"
#include "acceleration_disturbance.hpp"
#include "simple_disturbance.hpp"

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
  Disturbances(const SimulationConfig* sim_config, const int sat_id, const Structure* structure, const GlobalEnvironment* global_environment);
  /**
   * @fn ~Disturbances
   * @brief Destructor
   */
  virtual ~Disturbances();

  /**
   * @fn Update
   * @brief Update all disturbance calculation
   */
  void Update(const LocalEnvironment& local_environment, const Dynamics& dynamics, const SimTime* sim_time);
  /**
   * @fn LogSetup
   * @brief log setup for all disturbances
   */
  void LogSetup(Logger& logger);

  /**
   * @fn GetTorque
   * @brief Return total disturbance torque in the body frame [Nm]
   */
  inline libra::Vector<3> GetTorque_b_Nm() { return total_torque_b_Nm_; }

  /**
   * @fn GetTorque
   * @brief Return total disturbance force in the body frame [N]
   */
  inline libra::Vector<3> GetForce_b_N() { return total_force_b_N_; }

  /**
   * @fn GetTorque
   * @brief Return total disturbance acceleration in the inertial frame [m/s2]
   */
  inline libra::Vector<3> GetAcceleration_i_m_s2() { return total_acceleration_i_m_s2_; }

 private:
  std::string initialize_file_name_;  //!< Initialization file name

  std::vector<SimpleDisturbance*> disturbances_list_;  //!< List of disturbances
  Vector<3> total_torque_b_Nm_;                        //!< Total disturbance torque in the body frame [Nm]
  Vector<3> total_force_b_N_;                          //!< Total disturbance force in the body frame [N]

  vector<AccelerationDisturbance*> acceleration_disturbances_list_;  //!< List of acceleration disturbances
  Vector<3> total_acceleration_i_m_s2_;                              //!< Total disturbance acceleration in the inertial frame [m/s2]

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

#endif  // S2E_DISTURBANCES_DISTURBANCES_HPP_
