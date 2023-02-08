/**
 * @file dynamics.hpp
 * @brief Class to manage dynamics of spacecraft
 */
#ifndef S2E_DYNAMICS_DYNAMICS_H_
#define S2E_DYNAMICS_DYNAMICS_H_

#include <string>

#include "../library/math/Vector.hpp"
using libra::Vector;

#include <dynamics/attitude/initialize_attitude.hpp>
#include <dynamics/orbit/initialize_orbit.hpp>
#include <dynamics/thermal/initialize_node.hpp>
#include <dynamics/thermal/initialize_temperature.hpp>

#include "../environment/global/simulation_time.hpp"
#include "../environment/local/local_celestial_information.hpp"
#include "../simulation/simulation_configuration.hpp"
#include "../simulation/spacecraft/structure/structure.hpp"
#include "./orbit/orbit.hpp"
#include "./thermal/temperature.hpp"

class RelativeInformation;

/**
 * @class Dynamics
 * @brief Class to manage dynamics of spacecraft
 */
class Dynamics {
 public:
  /**
   * @fn Dynamics
   * @brief Constructor
   * @param [in] sim_config: Simulation config
   * @param [in] sim_time: Simulation time
   * @param [in] local_celes_info: Local celestial information
   * @param [in] sat_id: Spacecraft ID of the spacecraft
   * @param [in] structure: Structure of the spacecraft
   * @param [in] rel_info: Relative information
   */
  Dynamics(SimulationConfig* sim_config, const SimTime* sim_time, const LocalCelestialInformation* local_celes_info, const int sat_id,
           Structure* structure, RelativeInformation* rel_info = (RelativeInformation*)nullptr);
  /**
   * @fn ~Dynamics
   * @brief Destructor
   */
  virtual ~Dynamics();

  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] sim_config: Simulation config
   * @param [in] sim_time: Simulation time
   * @param [in] local_celes_info: Local celestial information
   * @param [in] sat_id: Spacecraft ID of the spacecraft
   * @param [in] structure: Structure of the spacecraft
   * @param [in] rel_info: Relative information
   */
  void Initialize(SimulationConfig* sim_config, const SimTime* sim_time, const LocalCelestialInformation* local_celes_info, const int sat_id,
                  Structure* structure, RelativeInformation* rel_info = (RelativeInformation*)nullptr);

  /**
   * @fn Update
   * @brief Update states of all dynamics calculations
   * @param [in] sim_time: Simulation time
   * @param [in] local_celes_info: Local celestial information
   */
  void Update(const SimTime* sim_time, const LocalCelestialInformation* local_celes_info);
  /**
   * @fn LogSetup
   * @brief Log setup for dynamics calculation
   */
  void LogSetup(Logger& logger);

  /**
   * @fn AddTorque_b
   * @brief Add input torque for the attitude dynamics propagation
   * @param [in] torue_b: Torque in the body fixed frame [Nm]
   */
  void AddTorque_b(Vector<3> torque_b);
  /**
   * @fn AddForce_b
   * @brief Add input force for the orbit dynamics propagation
   * @param [in] force_b: Force in the body fixed frame [N]
   */
  void AddForce_b(Vector<3> force_b);
  /**
   * @fn AddAcceleratione_b
   * @brief Add input acceleration for the orbit dynamics propagation
   * @param [in] acceleration_b: Force in the body fixed frame [N]
   */
  void AddAcceleration_i(Vector<3> acceleration_i);
  /**
   * @fn ClearForceTorque
   * @brief Clear force, acceleration, and torque for the dynamics propagation
   */
  void ClearForceTorque(void);

  /**
   * @fn GetAttitude
   * @brief Return Attitude class
   */
  inline const Attitude& GetAttitude() const { return *attitude_; }
  /**
   * @fn GetOrbit
   * @brief Return Orbit class
   */
  inline const Orbit& GetOrbit() const { return *orbit_; }
  /**
   * @fn GetTemperature
   * @brief Return Temperature class
   */
  inline const Temperature& GetTemperature() const { return *temperature_; }
  /**
   * @fn SetAttitude
   * @brief Return Attitude class to change the Attitude
   */
  inline Attitude& SetAttitude() const { return *attitude_; }

  /**
   * @fn GetPosition_i
   * @brief Return spacecraft position in the inertial frame [m]
   */
  virtual Vector<3> GetPosition_i() const;

  /**
   * @fn GetQuaternion_i2b
   * @brief Return spacecraft attitude quaternion from the inertial frame to the body fixed frame
   */
  virtual Quaternion GetQuaternion_i2b() const;

 private:
  Attitude* attitude_;          //!< Attitude dynamics
  Orbit* orbit_;                //!< Orbit dynamics
  Temperature* temperature_;    //!< Thermal dynamics
  const Structure* structure_;  //!< Structure information
};

#endif  // S2E_DYNAMICS_DYNAMICS_H_
