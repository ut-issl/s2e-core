/**
 * @file encke_orbit_propagation.hpp
 * @brief Class to propagate spacecraft orbit with Encke's method
 */

#ifndef S2E_DYNAMICS_ORBIT_ENCKE_ORBIT_PROPAGATION_HPP_
#define S2E_DYNAMICS_ORBIT_ENCKE_ORBIT_PROPAGATION_HPP_

#include "../../math_physics/math/ordinary_differential_equation.hpp"
#include "../../math_physics/orbit/kepler_orbit.hpp"
#include "orbit.hpp"

/**
 * @class EnckeOrbitPropagation
 * @brief Class to propagate spacecraft orbit with Encke's method
 */
class EnckeOrbitPropagation : public Orbit, public math::OrdinaryDifferentialEquation<6> {
 public:
  /**
   * @fn EnckeOrbitPropagation
   * @brief Constructor
   * @param [in] celestial_information: Celestial information
   * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
   * @param [in] propagation_step_s: Propagation step width [sec]
   * @param [in] current_time_jd: Current Julian day [day]
   * @param [in] position_i_m: Initial value of position in the inertial frame [m]
   * @param [in] velocity_i_m_s: Initial value of velocity in the inertial frame [m/s]
   * @param [in] error_tolerance: Error tolerance threshold
   */
  EnckeOrbitPropagation(const CelestialInformation* celestial_information, const double gravity_constant_m3_s2, const double propagation_step_s,
                        const double current_time_jd, const math::Vector<3> position_i_m, const math::Vector<3> velocity_i_m_s,
                        const double error_tolerance);
  /**
   * @fn ~EnckeOrbitPropagation
   * @brief Destructor
   */
  ~EnckeOrbitPropagation();

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] end_time_s: End time of simulation [sec]
   * @param [in] current_time_jd: Current Julian day [day]
   */
  virtual void Propagate(const double end_time_s, const double current_time_jd);

  // Override OrdinaryDifferentialEquation
  /**
   * @fn DerivativeFunction
   * @brief Right Hand Side of ordinary difference equation
   * @param [in] t: Time as independent variable
   * @param [in] state: Position and velocity as state vector
   * @param [out] rhs: Output of the function
   */
  virtual void DerivativeFunction(double t, const math::Vector<6>& state, math::Vector<6>& rhs);

 private:
  // General
  const double gravity_constant_m3_s2_;  //!< Gravity constant of the center body [m3/s2]
  const double error_tolerance_;         //!< Error tolerance ratio
  double propagation_step_s_;            //!< Propagation step width for RK4
  double propagation_time_s_;            //!< Simulation current time for numerical integration by RK4

  // reference orbit
  math::Vector<3> reference_position_i_m_;    //!< Reference orbit position in the inertial frame [m]
  math::Vector<3> reference_velocity_i_m_s_;  //!< Reference orbit velocity in the inertial frame [m/s]
  orbit::KeplerOrbit reference_kepler_orbit;  //!< Reference Kepler orbital element

  // difference orbit
  math::Vector<3> difference_position_i_m_;    //!< Difference orbit position in the inertial frame [m]
  math::Vector<3> difference_velocity_i_m_s_;  //!< Difference orbit velocity in the inertial frame [m/s]

  // functions
  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] current_time_jd: Current Julian day [day]
   * @param [in] reference_position_i_m: Initial value of reference orbit position in the inertial frame [m]
   * @param [in] reference_velocity_i_m_s: Initial value of reference orbit position in the inertial frame [m]
   */
  void Initialize(const double current_time_jd, const math::Vector<3> reference_position_i_m, const math::Vector<3> reference_velocity_i_m_s);
  /**
   * @fn UpdateSatOrbit
   * @brief Update satellite orbit
   */
  void UpdateSatOrbit();
  /**
   * @fn CalcQFunction
   * @brief Calculate Q function
   * @param [in] difference_position_i_m: Difference of position in the inertial frame [m]
   */
  double CalcQFunction(const math::Vector<3> difference_position_i_m);
};

#endif  // S2E_DYNAMICS_ORBIT_ENCKE_ORBIT_PROPAGATION_HPP_
