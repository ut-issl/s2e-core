/**
 * @file kepler_orbit.hpp
 * @brief Class to calculate Kepler orbit calculation
 */

#ifndef S2E_LIBRARY_ORBIT_KEPLER_ORBIT_HPP_
#define S2E_LIBRARY_ORBIT_KEPLER_ORBIT_HPP_

#include "../math/matrix.hpp"
#include "../math/vector.hpp"
#include "./orbital_elements.hpp"

namespace s2e::orbit {

/**
 * @class KeplerOrbit
 * @brief Class to calculate Kepler orbit calculation
 */
class KeplerOrbit {
 public:
  /**
   * @fn KeplerOrbit
   * @brief Default Constructor
   */
  KeplerOrbit();
  /**
   * @fn KeplerOrbit
   * @brief Constructor
   * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
   * @param [in] oe: Orbital elements
   */
  KeplerOrbit(const double gravity_constant_m3_s2, const OrbitalElements oe);
  /**
   * @fn ~KeplerOrbit
   * @brief Destructor
   */
  ~KeplerOrbit();

  /**
   * @fn CalcOrbit
   * @brief Calculate position and velocity with Kepler orbit propagation
   * @param [in] time_jday: Time expressed as Julian day [day]
   */
  void CalcOrbit(double time_jday);

  /**
   * @fn GetPosition_i_m
   * @brief Return position vector in the inertial frame [m]
   */
  inline const math::Vector<3> GetPosition_i_m() const { return position_i_m_; }
  /**
   * @fn GetVelocity_i_m_s
   * @brief Return velocity vector in the inertial frame [m/s]
   */
  inline const math::Vector<3> GetVelocity_i_m_s() const { return velocity_i_m_s_; }

 protected:
  math::Vector<3> position_i_m_;    //!< Position vector in the inertial frame [m]
  math::Vector<3> velocity_i_m_s_;  //!< Velocity vector in the inertial frame [m/s]

 private:
  double gravity_constant_m3_s2_;        //!< Gravity constant of the center body [m3/s2]
  OrbitalElements oe_;                   //!< Orbital elements
  double mean_motion_rad_s_;             //!< Mean motion of the orbit [rad/s]
  math::Matrix<3, 3> dcm_inplane_to_i_;  //!< Direction cosine matrix from the in-plane frame to the inertial frame

  /**
   * @fn CalcConstKeplerMotion
   * @brief Calculate constants for kepler motion calculation
   */
  void CalcConstKeplerMotion();
  /**
   * @fn SolveKeplerFirstOrder
   * @brief Solve Kepler Equation with the first order approximation
   * @param [in] eccentricity: Eccentricity
   * @param [in] mean_anomaly_rad: Mean motion of the orbit [rad/s]
   * @param [in] angle_limit_rad: Limit of angle error for the approximation
   * @param [in] iteration_limit: Limit of iteration
   */
  double SolveKeplerFirstOrder(const double eccentricity, const double mean_anomaly_rad, const double angle_limit_rad, const int iteration_limit);
  /**
   * @fn SolveKeplerNewtonMethod
   * @brief Solve Kepler Equation with the Newton Method
   * @param [in] eccentricity: Eccentricity
   * @param [in] mean_anomaly_rad: Mean motion of the orbit [rad/s]
   * @param [in] angle_limit_rad: Limit of angle error for the approximation
   * @param [in] iteration_limit: Limit of iteration
   */
  double SolveKeplerNewtonMethod(const double eccentricity, const double mean_anomaly_rad, const double angle_limit_rad, const int iteration_limit);
};

}  // namespace s2e::orbit

#endif  // S2E_LIBRARY_ORBIT_KEPLER_ORBIT_HPP_
