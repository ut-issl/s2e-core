/**
 * @file relative_orbit_sabatini.hpp
 * @brief Functions to calculate sabatini STM for relative orbit
 */

#ifndef S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_SABATINI_HPP_
#define S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_SABATINI_HPP_

#include "../math/matrix.hpp"
#include "../math/vector.hpp"
#include "./orbital_elements.hpp"

namespace orbit {

/**
 * @class RelativeOrbitSabatini
 * @brief Class to calculate Sabatini relative orbital STM
 */
class RelativeOrbitSabatini {
 public:
  /**
   * @fn RelativeOrbitSabatini
   * @param [in] position_i_m: Initial value of position in the inertial frame [m]
   * @param [in] velocity_i_m_s: Initial value of velocity in the inertial frame [m/s]
   * @brief Default Constructor
   */
  RelativeOrbitSabatini(math::Vector<3> position_i_m, math::Vector<3> velocity_i_m_s);
  /**
   * @fn ~RelativeOrbitSabatini
   * @brief Destructor
   */
  ~RelativeOrbitSabatini();

  /**
   * @fn CalculateSystemMatrix
   * @brief Calculate system matrix for relative orbit
   * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
   * @param [in] f_ref_rad: True anomaly of the reference satellite [rad]
   * @param [in] reference_oe: Orbital elements of reference satellite
   */
  math::Matrix<6, 6> CalcSabatiniSystemMatrix(double gravity_constant_m3_s2, double f_ref_rad, OrbitalElements* reference_oe);

 private:
  double initial_orbit_radius_m_;         //!< Initial orbit radius [m]
  double initial_angular_momentum_m2_s_;  //!< Initial angular momentum [m2/s]
};

}  // namespace orbit

#endif  // S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_SABATINI_HPP_
