/**
 * @file encke_orbit_propagation.hpp
 * @brief Class to propagate spacecraft orbit with Encke's method
 */

#ifndef S2E_DYNAMICS_ORBIT_ENCKE_ORBIT_PROPAGATION_H_
#define S2E_DYNAMICS_ORBIT_ENCKE_ORBIT_PROPAGATION_H_

#include "../../library/Orbit/KeplerOrbit.h"
#include "../../library/math/ODE.hpp"
#include "orbit.hpp"

/**
 * @class EnckeOrbitPropagation
 * @brief Class to propagate spacecraft orbit with Encke's method
 */
class EnckeOrbitPropagation : public Orbit, public libra::ODE<6> {
 public:
  /**
   * @fn EnckeOrbitPropagation
   * @brief Constructor
   * @param [in] celes_info: Celestial information
   * @param [in] mu_m3_s2: Gravity constant of the center body [m3/s2]
   * @param [in] prop_step_s: Propagation step width [sec]
   * @param [in] current_jd: Current Julian day [day]
   * @param [in] init_position_i_m: Initial value of position in the inertial frame [m]
   * @param [in] init_velocity_i_m_s: Initial value of velocity in the inertial frame [m/s]
   * @param [in] error_tolerance: Error tolerance threshold
   */
  EnckeOrbitPropagation(const CelestialInformation* celes_info, const double mu_m3_s2, const double prop_step_s, const double current_jd,
                        const Vector<3> init_position_i_m, const Vector<3> init_velocity_i_m_s, const double error_tolerance);
  /**
   * @fn ~EnckeOrbitPropagation
   * @brief Destructor
   */
  ~EnckeOrbitPropagation();

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] endtime: End time of simulation [sec]
   * @param [in] current_jd: Current Julian day [day]
   */
  virtual void Propagate(double endtime, double current_jd);

  // Override ODE
  /**
   * @fn RHS
   * @brief Right Hand Side of ordinary difference equation
   * @param [in] t: Time as independent variable
   * @param [in] state: Position and velocity as state vector
   * @param [out] rhs: Output of the function
   */
  virtual void RHS(double t, const Vector<6>& state, Vector<6>& rhs);

 private:
  // General
  const double mu_m3_s2_;         //!< Gravity constant of the center body [m3/s2]
  const double error_tolerance_;  //!< Error tolerance ratio
  double prop_step_s_;            //!< Propagation step width for RK4
  double prop_time_s_;            //!< Simulation current time for numerical integration by RK4

  // reference orbit
  Vector<3> ref_position_i_m_;    //!< Reference orbit position in the inertial frame [m]
  Vector<3> ref_velocity_i_m_s_;  //!< Reference orbit velocity in the inertial frame [m/s]
  KeplerOrbit ref_kepler_orbit;   //!< Reference Kepler orbital element

  // difference orbit
  Vector<3> diff_position_i_m_;    //!< Difference orbit position in the inertial frame [m]
  Vector<3> diff_velocity_i_m_s_;  //!< Difference orbit velocity in the inertial frame [m/s]

  // functions
  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] current_jd: Current Julian day [day]
   * @param [in] init_ref_position_i_m: Initial value of reference orbit position in the inertial frame [m]
   * @param [in] init_ref_velocity_i_m_s: Initial value of reference orbit position in the inertial frame [m]
   */
  void Initialize(double current_jd, Vector<3> init_ref_position_i_m, Vector<3> init_ref_velocity_i_m_s);
  /**
   * @fn UpdateSatOrbit
   * @brief Update satellite orbit
   */
  void UpdateSatOrbit();
  /**
   * @fn CalcQFunction
   * @brief Calculate Q function
   * @param [in] diff_pos_i: Difference of position in the inertial frame [m]
   */
  double CalcQFunction(Vector<3> diff_pos_i);
};

#endif  // S2E_DYNAMICS_ORBIT_ENCKE_ORBIT_PROPAGATION_H_
