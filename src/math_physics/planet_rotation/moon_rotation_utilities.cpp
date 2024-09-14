/**
 * @file moon_rotation_utilities.cpp
 * @brief Functions to calculate the moon rotation frame conversion
 * @note Ref: A Standardized Lunar Coordinate System for the Lunar Reconnaissance Orbiter and Lunar Datasets
 *            https://lunar.gsfc.nasa.gov/library/LunCoordWhitePaper-10-08.pdf
 *            https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de430_moon_coord.pdf
 */

#include "moon_rotation_utilities.hpp"

#include <math_physics/math/constants.hpp>

namespace s2e::planet_rotation {

s2e::math::Matrix<3, 3> CalcDcmEciToPrincipalAxis(const s2e::math::Vector<3> moon_position_eci_m, const s2e::math::Vector<3> moon_velocity_eci_m_s) {
  s2e::math::Matrix<3, 3> dcm_eci2me = CalcDcmEciToMeanEarth(moon_position_eci_m, moon_velocity_eci_m_s);
  s2e::math::Matrix<3, 3> dcm_me2pa = CalcDcmMeanEarthToPrincipalAxis();

  return dcm_me2pa * dcm_eci2me;
}

s2e::math::Matrix<3, 3> CalcDcmEciToMeanEarth(const s2e::math::Vector<3> moon_position_eci_m, const s2e::math::Vector<3> moon_velocity_eci_m_s) {
  s2e::math::Vector<3> me_ex_eci = -1.0 * moon_position_eci_m.CalcNormalizedVector();

  s2e::math::Vector<3> moon_orbit_norm = s2e::math::OuterProduct(moon_position_eci_m, moon_velocity_eci_m_s);
  s2e::math::Vector<3> me_ez_eci = moon_orbit_norm.CalcNormalizedVector();

  s2e::math::Vector<3> me_ey_eci = s2e::math::OuterProduct(me_ez_eci, me_ex_eci);

  s2e::math::Matrix<3, 3> dcm_eci_to_me;
  for (size_t i = 0; i < 3; i++) {
    dcm_eci_to_me[0][i] = me_ex_eci[i];
    dcm_eci_to_me[1][i] = me_ey_eci[i];
    dcm_eci_to_me[2][i] = me_ez_eci[i];
  }

  return dcm_eci_to_me;
}

s2e::math::Matrix<3, 3> CalcDcmMeanEarthToPrincipalAxis() {
  // The correction values between DE430 Principal Axis and Mean Earth frame
  const double theta_x_rad = 0.285 * s2e::math::arcsec_to_rad;
  const double theta_y_rad = 78.580 * s2e::math::arcsec_to_rad;
  const double theta_z_rad = 67.573 * s2e::math::arcsec_to_rad;

  s2e::math::Matrix<3, 3> dcm_me_pa =
      s2e::math::MakeRotationMatrixZ(theta_z_rad) * s2e::math::MakeRotationMatrixY(theta_y_rad) * s2e::math::MakeRotationMatrixX(theta_x_rad);

  return dcm_me_pa;
}

}  // namespace s2e::planet_rotation
