/**
 * @file attitude_with_cantilever_vibration.hpp
 * @brief Class to calculate spacecraft attitude with cantilever vibration
 */

#ifndef S2E_DYNAMICS_ATTITUDE_ATTITUDE_WITH_CANTILEVER_VIBRATION_HPP_
#define S2E_DYNAMICS_ATTITUDE_ATTITUDE_WITH_CANTILEVER_VIBRATION_HPP_

#include <math_physics/numerical_integration/numerical_integrator_manager.hpp>
#include <utilities/macros.hpp>

#include "attitude.hpp"
#include "ode_attitude_with_cantilever_vibration.hpp"

namespace s2e::dynamics::attitude {

/**
 * @class AttitudeWithCantileverVibration
 * @brief Class to calculate spacecraft attitude with cantilever vibration
 */
class AttitudeWithCantileverVibration : public Attitude {
 public:
  /**
   * @fn AttitudeWithCantileverVibration
   * @brief Constructor
   * @param [in] angular_velocity_b_rad_s: Initial value of spacecraft angular velocity of the body fixed frame [rad/s]
   * @param [in] quaternion_i2b: Initial value of attitude quaternion from the inertial frame to the body fixed frame
   * @param [in] inertia_tensor_kgm2: Initial value of inertia tensor of the spacecraft [kg m^2]
   * @param [in] inertia_tensor_cantilever_kgm2: Initial value of inertia tensor of the cantilever [kg m^2]
   * @param [in] damping_ratio_cantilever: Initial value of damping ratio of the cantilever []
   * @param [in] intrinsic_angular_velocity_cantilever_rad_s: Initial value of intrinsic angular velocity [rad/s]
   * @param [in] torque_b_Nm: Initial torque acting on the spacecraft in the body fixed frame [Nm]
   * @param [in] propagation_step_s: Initial value of propagation step width [sec]
   * @param [in] simulation_object_name: Simulation object name for Monte-Carlo simulation
   */
  AttitudeWithCantileverVibration(const math::Vector<3>& angular_velocity_b_rad_s, const math::Quaternion& quaternion_i2b,
                                  const math::Matrix<3, 3>& inertia_tensor_kgm2, const math::Matrix<3, 3>& inertia_tensor_cantilever_kgm2,
                                  const double damping_ratio_cantilever, const double intrinsic_angular_velocity_cantilever_rad_s,
                                  const math::Vector<3>& torque_b_Nm, const double propagation_step_s,
                                  const std::string& simulation_object_name = "attitude");
  /**
   * @fn ~AttitudeWithCantileverVibration
   * @brief Destructor
   */
  ~AttitudeWithCantileverVibration();

  /**
   * @fn Propagate
   * @brief Attitude propagation
   * @param [in] end_time_s: Propagation endtime [sec]
   */
  virtual void Propagate(const double end_time_s);

  // Override logger::ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of logger::ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of logger::ILoggable
   */
  virtual std::string GetLogValue() const;

  /**
   * @fn SetParameters
   * @brief Set parameters for Monte-Carlo simulation
   * @param [in] mc_simulator: Monte-Carlo simulation executor
   */
  virtual void SetParameters(const simulation::MonteCarloSimulationExecutor& mc_simulator);

 private:
  double current_propagation_time_s_;                       //!< current time [sec]
  math::Vector<3> angular_velocity_cantilever_rad_s_{0.0};  //!< Angular velocity of the cantilever with respect to the body frame [rad/s]
  math::Vector<3> euler_angular_cantilever_rad_{0.0};       //!< Euler angle of the cantilever with respect to the body frame [rad/s]

  AttitudeWithCantileverVibrationOde attitude_ode_;
  s2e::numerical_integration::NumericalIntegratorManager<13> numerical_integrator_;
};

}  // namespace s2e::dynamics::attitude

#endif  // S2E_DYNAMICS_ATTITUDE_ATTITUDE_WITH_CANTILEVER_VIBRATION_HPP_
