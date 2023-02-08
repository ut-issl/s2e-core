/**
 * @file rk4_orbit_propagation.hpp
 * @brief Class to propagate spacecraft orbit with Runge-Kutta-4 method
 */

#ifndef S2E_DYNAMICS_ORBIT_RK4_ORBIT_PROPAGATION_HPP_
#define S2E_DYNAMICS_ORBIT_RK4_ORBIT_PROPAGATION_HPP_

#include <environment/global/celestial_information.hpp>
#include <library/math/ordinary_differential_equation.hpp>

#include "orbit.hpp"

/**
 * @class Rk4OrbitPropagation
 * @brief Class to propagate spacecraft orbit with Runge-Kutta-4 method
 */
class Rk4OrbitPropagation : public Orbit, public libra::ODE<6> {
 private:
  static const int N = 6;  //!< Degrees of freedom in 3D space
  double mu;               //!< Gravity constant [m3/s2]

 public:
  /**
   * @fn Rk4OrbitPropagation
   * @brief Constructor
   * @param [in] celes_info: Celestial information
   * @param [in] mu: Gravity constant [m3/s2]
   * @param [in] timestep: Step width [sec]
   * @param [in] init_position: Initial value of position in the inertial frame [m]
   * @param [in] init_velocity: Initial value of velocity in the inertial frame [m/s]
   * @param [in] init_time: Initial time [sec]
   */
  Rk4OrbitPropagation(const CelestialInformation* celes_info, double mu, double timestep, Vector<3> init_position, Vector<3> init_velocity,
                      double init_time = 0);
  /**
   * @fn ~Rk4OrbitPropagation
   * @brief Destructor
   */
  ~Rk4OrbitPropagation();

  // Override ODE
  /**
   * @fn RHS
   * @brief Right Hand Side of ordinary difference equation
   * @param [in] t: Time as independent variable
   * @param [in] state: Position and velocity as state vector
   * @param [out] rhs: Output of the function
   */
  virtual void RHS(double t, const Vector<N>& state, Vector<N>& rhs);

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] endtime: End time of simulation [sec]
   * @param [in] current_jd: Current Julian day [day]
   */
  virtual void Propagate(double endtime, double current_jd);

  /**
   * @fn AddPositionOffset
   * @brief Shift the position of the spacecraft
   * @note Is this really needed?
   * @param [in] offset_i: Offset vector in the inertial frame [m]
   */
  virtual void AddPositionOffset(Vector<3> offset_i);

 private:
  double prop_time_;  //!< Simulation current time for numerical integration by RK4 [sec]
  double prop_step_;  //!< Step width for RK4 [sec]

  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] init_position: Initial value of position in the inertial frame [m]
   * @param [in] init_velocity: Initial value of velocity in the inertial frame [m/s]
   * @param [in] init_time: Initial time [sec]
   */
  void Initialize(Vector<3> init_position, Vector<3> init_velocity, double init_time = 0);
};

#endif  // S2E_DYNAMICS_ORBIT_RK4_ORBIT_PROPAGATION_HPP_
