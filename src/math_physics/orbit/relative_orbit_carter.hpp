/**
 * @file relative_orbit_carter.hpp
 * @brief Functions to calculate Carter's STM for relative orbit
 */

#ifndef S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_CARTER_HPP_
#define S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_CARTER_HPP_

#include "../math/matrix.hpp"
#include "./orbital_elements.hpp"

namespace orbit {

/**
 * @class RelativeOrbitCarter
 * @brief Class to calculate Yamanaka-Ankersen relative orbital STM
 */
class RelativeOrbitCarter {
 public:
  /**
   * @fn RelativeOrbitCarter
   * @brief Default Constructor
   */
  RelativeOrbitCarter();
  /**
   * @fn ~RelativeOrbitCarter
   * @brief Destructor
   */
  ~RelativeOrbitCarter();

  /**
   * @fn CalculateInitialInverseMatrix
   * @brief Calculate position and velocity with Kepler orbit propagation
   * @param [in] orbit_radius_m: Orbit radius [m]
   * @param [in] f_ref_rad: True anomaly of the reference satellite [rad]
   * @param [in] reference_oe: Orbital elements of reference satellite
   */
  void CalculateInitialInverseMatrix(double orbit_radius_m, double f_ref_rad, OrbitalElements* reference_oe);

  /**
   * @fn CalculateSTM
   * @brief Calculate position and velocity with Kepler orbit propagation
   * @param [in] orbit_radius_m: Orbit radius [m]
   * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
   * @param [in] f_ref_rad: True anomaly of the reference satellite [rad]
   * @param [in] reference_oe: Orbital elements of reference satellite
   */
  math::Matrix<6, 6> CalculateSTM(double orbit_radius_m, double gravity_constant_m3_s2, double f_ref_rad, OrbitalElements* reference_oe);

  /**
   * @fn GetInitialInverseMatrix
   * @brief Return initial inverse matrix
   */
  inline const math::Matrix<6, 6> GetInitialInverseMatrix() const { return initial_inverse_matrix_; }

 private:
  math::Matrix<6, 6> initial_inverse_matrix_{0.0};  //!< Gravity constant of the center body [m3/s2]
};

}  // namespace orbit

#endif  // S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_MODEL_HPP_
