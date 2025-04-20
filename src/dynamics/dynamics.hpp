/**
 * @file dynamics.hpp
 * @brief Class to manage dynamics of spacecraft
 */
#ifndef S2E_DYNAMICS_DYNAMICS_HPP_
#define S2E_DYNAMICS_DYNAMICS_HPP_

#include <components/ports/power_port_provider.hpp>
#include <string>

#include "../environment/global/simulation_time.hpp"
#include "../environment/local/local_environment.hpp"
#include "../math_physics/math/vector.hpp"
#include "../simulation/multiple_spacecraft/relative_information.hpp"
#include "../simulation/simulation_configuration.hpp"
#include "../simulation/spacecraft/structure/structure.hpp"
#include "dynamics/attitude/initialize_attitude.hpp"
#include "dynamics/orbit/initialize_orbit.hpp"
#include "dynamics/thermal/node.hpp"
#include "dynamics/thermal/temperature.hpp"

namespace s2e::simulation {
class RelativeInformation;
}
namespace s2e::environment {
class LocalEnvironment;
}

namespace s2e::dynamics {

/**
 * @class Dynamics
 * @brief Class to manage dynamics of spacecraft
 */
class Dynamics {
 public:
  /**
   * @fn Dynamics
   * @brief Constructor
   * @param [in] simulation_configuration: Simulation configuration
   * @param [in] simulation_time: Simulation time
   * @param [in] local_celestial_information: Local celestial information
   * @param [in] spacecraft_id: Spacecraft ID of the spacecraft
   * @param [in] structure: spacecraft::Structure of the spacecraft
   * @param [in] relative_information: Relative information
   */
  Dynamics(const simulation::SimulationConfiguration* simulation_configuration, const environment::SimulationTime* simulation_time,
           const environment::LocalEnvironment* local_environment, const int spacecraft_id, spacecraft::Structure* structure,
           simulation::RelativeInformation* relative_information = (simulation::RelativeInformation*)nullptr,
           const s2e::components::PowerPortProvider* power_port_provider = (s2e::components::PowerPortProvider*)nullptr);
  /**
   * @fn ~Dynamics
   * @brief Destructor
   */
  virtual ~Dynamics();

  /**
   * @fn Update
   * @brief Update states of all dynamics calculations
   * @param [in] simulation_time: Simulation time
   * @param [in] local_celestial_information: Local celestial information
   */
  void Update(const environment::SimulationTime* simulation_time, const environment::LocalCelestialInformation* local_celestial_information);

  /**
   * @fn LogSetup
   * @brief Log setup for dynamics calculation
   */
  void LogSetup(logger::Logger& logger);

  /**
   * @fn AddTorque_b_Nm
   * @brief Add input torque for the attitude dynamics propagation
   * @param [in] torque_b_Nm: Torque in the body fixed frame [Nm]
   */
  inline void AddTorque_b_Nm(math::Vector<3> torque_b_Nm) { attitude_->AddTorque_b_Nm(torque_b_Nm); }
  /**
   * @fn AddForce_b_N
   * @brief Add input force for the orbit dynamics propagation
   * @param [in] force_b_N: Force in the body fixed frame [N]
   */
  inline void AddForce_b_N(math::Vector<3> force_b_N) {
    orbit_->AddForce_b_N(force_b_N, attitude_->GetQuaternion_i2b(), structure_->GetKinematicsParameters().GetMass_kg());
  }
  /**
   * @fn AddAcceleration_i_m_s2
   * @brief Add input acceleration for the orbit dynamics propagation
   * @param [in] acceleration_i_m_s2: Acceleration in the inertial fixed frame [N]
   */
  inline void AddAcceleration_i_m_s2(math::Vector<3> acceleration_i_m_s2) { orbit_->AddAcceleration_i_m_s2(acceleration_i_m_s2); }

  /**
   * @fn ClearForceTorque
   * @brief Clear force, acceleration, and torque for the dynamics propagation
   */
  void ClearForceTorque(void);

  /**
   * @fn SetPowerPortProvider
   * @brief Set Power Port Provider for dynamics and temperature
   */
  void SetPowerPortProvider(s2e::components::PowerPortProvider* power_port_provider);


  /**
   * @fn GetAttitude
   * @brief Return Attitude class
   */
  inline const attitude::Attitude& GetAttitude() const { return *attitude_; }
  /**
   * @fn GetOrbit
   * @brief Return Orbit class
   */
  inline const orbit::Orbit& GetOrbit() const { return *orbit_; }
  /**
   * @fn GetTemperature
   * @brief Return Temperature class
   */
  inline const thermal::Temperature& GetTemperature() const { return *temperature_; }

 private:
  attitude::Attitude* attitude_;                                   //!< Attitude dynamics
  orbit::Orbit* orbit_;                                            //!< Orbit dynamics
  thermal::Temperature* temperature_;                              //!< Thermal dynamics
  const spacecraft::Structure* structure_;                         //!< Structure information
  const environment::LocalEnvironment* local_environment_;         //!< Local environment
  const s2e::components::PowerPortProvider* power_port_provider_;  //!< Power port provider to get power consumption

  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] simulation_configuration: Simulation configuration
   * @param [in] simulation_time: Simulation time
   * @param [in] local_celestial_information: Local celestial information
   * @param [in] spacecraft_id: Spacecraft ID of the spacecraft
   * @param [in] structure: Structure of the spacecraft
   * @param [in] relative_information: Relative information
   * @param [in] power_port_provider: Power port provider
   */
  void Initialize(const simulation::SimulationConfiguration* simulation_configuration, const environment::SimulationTime* simulation_time,
                  const int spacecraft_id, spacecraft::Structure* structure,
                  simulation::RelativeInformation* relative_information = (simulation::RelativeInformation*)nullptr,
                  const s2e::components::PowerPortProvider* power_port_provider = (s2e::components::PowerPortProvider*)nullptr);
};

}  // namespace s2e::dynamics

#endif  // S2E_DYNAMICS_DYNAMICS_HPP_
