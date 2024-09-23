/**
 * @file relative_orbit_yamanaka_ankersen.hpp
 * @brief Functions to calculate Yamanaka-ANkersen STM for relative orbit
 */

#ifndef S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_YAMANAKA_ANKERSEN_HPP_
#define S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_YAMANAKA_ANKERSEN_HPP_

#include "../math/matrix.hpp"
#include "./orbital_elements.hpp"

namespace orbit {

/**
 * @class RelativeOrbitYamanakaAnkersen
 * @brief Class to calculate Yamanaka-Ankersen relative orbital STM
 */
class RelativeOrbitYamanakaAnkersen {
 public:
  /**
   * @fn RelativeOrbitYamanakaAnkersen
   * @brief Default Constructor
   */
  RelativeOrbitYamanakaAnkersen();
  /**
   * @fn ~RelativeOrbitYamanakaAnkersen
   * @brief Destructor
   */
  ~RelativeOrbitYamanakaAnkersen();

  /**
   * @fn CalculateInitialInverseMatrix
   * @brief Calculate position and velocity with Kepler orbit propagation
   * @param [in] f_ref_rad: True anomaly of the reference satellite [rad]
   * @param [in] reference_oe: Orbital elements of reference satellite
   */
  void CalculateInitialInverseMatrix(double f_ref_rad, OrbitalElements* reference_oe);

  /**
   * @fn CalculateSTM
   * @brief Calculate position and velocity with Kepler orbit propagation
   * @param [in] gravity_constant_m3_s2: Gravity constant of the center body [m3/s2]
   * @param [in] elapsed_time_s: Elapsed time [s]
   * @param [in] f_ref_rad: True anomaly of the reference satellite [rad]
   * @param [in] reference_oe: Orbital elements of reference satellite
   */
  math::Matrix<6, 6> CalculateSTM(double gravity_constant_m3_s2, double elapsed_time_s, double f_ref_rad, OrbitalElements* reference_oe);

 private:
  math::Matrix<6, 6> initial_inverse_matrix_{0.0};  //!< Gravity constant of the center body [m3/s2]
};

}  // namespace orbit

#endif  // S2E_LIBRARY_ORBIT_RELATIVE_ORBIT_MODEL_HPP_
