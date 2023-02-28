/**
 * @file attitude_rk4.hpp
 * @brief Class to calculate spacecraft attitude with Runge-Kutta method
 */

#ifndef S2E_DYNAMICS_ATTITUDE_ATTITUDE_RK4_HPP_
#define S2E_DYNAMICS_ATTITUDE_ATTITUDE_RK4_HPP_

#include "attitude.hpp"

/**
 * @class AttitudeRk4
 * @brief Class to calculate spacecraft attitude with Runge-Kutta method
 */
class AttitudeRk4 : public Attitude {
 public:
  /**
   * @fn AttitudeRk4
   * @brief Constructor
   * @param [in] angular_velocity_b_rad_s: Initial value of spacecraft angular velocity of the body fixed frame [rad/s]
   * @param [in] quaternion_i2b: Initial value of attitude quaternion from the inertial frame to the body fixed frame
   * @param [in] inertia_tensor_kgm2: Initial value of inertia tensor of the spacecraft [kg m^2]
   * @param [in] torque_b_Nm: Initial torque acting on the spacecraft in the body fixed frame [Nm]
   * @param [in] propagation_step_s: Initial value of propagation step width [sec]
   * @param [in] simulation_object_name: Simulation object name for Monte-Carlo simulation
   */
  AttitudeRk4(const libra::Vector<3>& angular_velocity_b_rad_s, const libra::Quaternion& quaternion_i2b,
              const libra::Matrix<3, 3>& inertia_tensor_kgm2, const libra::Vector<3>& torque_b_Nm, const double propagation_step_s,
              const std::string& simulation_object_name = "Attitude");
  /**
   * @fn ~AttitudeRk4
   * @brief Destructor
   */
  ~AttitudeRk4();

  /**
   * @fn Propagate
   * @brief Attitude propagation
   * @param [in] end_time_s: Propagation endtime [sec]
   */
  virtual void Propagate(const double end_time_s);

  /**
   * @fn SetParameters
   * @brief Set parameters for Monte-Carlo simulation
   * @param [in] mc_simulator: Monte-Carlo simulation executor
   */
  virtual void SetParameters(const MCSimExecutor& mc_simulator);

 private:
  double current_propagation_time_s_;  //!< current time [sec]

  /**
   * @fn CalcAngularVelocityMatrix
   * @brief Generate angular velocity matrix for kinematics calculation
   * @param [in] angular_velocity_b_rad_s: Angular velocity [rad/s]
   */
  libra::Matrix<4, 4> CalcAngularVelocityMatrix(libra::Vector<3> angular_velocity_b_rad_s);
  /**
   * @fn AttitudeDynamicsAndKinematics
   * @brief Dynamics equation with kinematics
   * @param [in] x: State vector (angular velocity and quaternion)
   * @param [in] t: Unused TODO: remove?
   */
  libra::Vector<7> AttitudeDynamicsAndKinematics(libra::Vector<7> x, double t);
  /**
   * @fn RungeKuttaOneStep
   * @brief Equation for one step of Runge-Kutta method
   * @param [in] t: Unused TODO: remove?
   * @param [in] dt: Step width [sec]
   */
  void RungeKuttaOneStep(double t, double dt);
};

#endif  // S2E_DYNAMICS_ATTITUDE_ATTITUDE_RK4_HPP_
