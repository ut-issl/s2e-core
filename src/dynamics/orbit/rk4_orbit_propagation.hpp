/**
 * @file rk4_orbit_propagation.hpp
 * @brief Class to propagate spacecraft orbit with Runge-Kutta-4 method
 */

#ifndef S2E_DYNAMICS_ORBIT_RK4_ORBIT_PROPAGATION_HPP_
#define S2E_DYNAMICS_ORBIT_RK4_ORBIT_PROPAGATION_HPP_

#include <environment/global/celestial_information.hpp>
#include <math_physics/math/ordinary_differential_equation.hpp>

#include "orbit.hpp"

namespace s2e::dynamics::orbit {

/**
 * @class Rk4OrbitPropagation
 * @brief Class to propagate spacecraft orbit with Runge-Kutta-4 method
 */
class Rk4OrbitPropagation : public Orbit, public math::OrdinaryDifferentialEquation<6> {
 public:
  /**
   * @fn Rk4OrbitPropagation
   * @brief Constructor
   * @param [in] celestial_information: Celestial information
   * @param [in] gravity_constant_m3_s2: Gravity constant [m3/s2]
   * @param [in] time_step_s: Step width [sec]
   * @param [in] position_i_m: Initial value of position in the inertial frame [m]
   * @param [in] velocity_i_m_s: Initial value of velocity in the inertial frame [m/s]
   * @param [in] initial_time_s: Initial time [sec]
   */
  Rk4OrbitPropagation(const CelestialInformation* celestial_information, double gravity_constant_m3_s2, double time_step_s,
                      math::Vector<3> position_i_m, math::Vector<3> velocity_i_m_s, double initial_time_s = 0);
  /**
   * @fn ~Rk4OrbitPropagation
   * @brief Destructor
   */
  ~Rk4OrbitPropagation();

  // Override OrdinaryDifferentialEquation
  /**
   * @fn DerivativeFunction
   * @brief Right Hand Side of ordinary difference equation
   * @param [in] t: Time as independent variable
   * @param [in] state: Position and velocity as state vector
   * @param [out] rhs: Output of the function
   */
  virtual void DerivativeFunction(double t, const math::Vector<6>& state, math::Vector<6>& rhs);

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] end_time_s: End time of simulation [sec]
   * @param [in] current_time_jd: Current Julian day [day]
   */
  virtual void Propagate(const double end_time_s, const double current_time_jd);

 private:
  double gravity_constant_m3_s2_;  //!< Gravity constant [m3/s2]
  double propagation_time_s_;      //!< Simulation current time for numerical integration by RK4 [sec]
  double propagation_step_s_;      //!< Step width for RK4 [sec]

  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] position_i_m: Initial value of position in the inertial frame [m]
   * @param [in] velocity_i_m_s: Initial value of velocity in the inertial frame [m/s]
   * @param [in] initial_time_s: Initial time [sec]
   */
  void Initialize(math::Vector<3> position_i_m, math::Vector<3> velocity_i_m_s, double initial_time_s = 0);
};

}  // namespace s2e::dynamics::orbit

#endif  // S2E_DYNAMICS_ORBIT_RK4_ORBIT_PROPAGATION_HPP_
