/**
 * @file moon_rotation.hpp
 * @brief Class to calculate the moon rotation
 * @note Ref: A Standardized Lunar Coordinate System for the Lunar Reconnaissance Orbiter and Lunar Datasets
 *            https://lunar.gsfc.nasa.gov/library/LunCoordWhitePaper-10-08.pdf
 *            https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de430_moon_coord.pdf
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_MOON_ROTATION_HPP_
#define S2E_ENVIRONMENT_GLOBAL_MOON_ROTATION_HPP_

#include "library/math/matrix.hpp"
#include "library/math/vector.hpp"

/**
 * @class MoonRotation
 * @brief Class to calculate the moon rotation
 */
class MoonRotation {
 public:
  /**
   * @fn MoonRotation
   * @brief Constructor
   */
  MoonRotation();

  /**
   * @fn Update
   * @brief Update rotation
   * @param [in] julian_date: Julian date
   */
  void Update(const libra::Vector<3> moon_position_eci_m, const libra::Vector<3> moon_velocity_eci_m_s);

  /**
   * @fn GetDcmJ2000ToMcmf
   * @brief Return the DCM between J2000 inertial frame and the Moon Centered Moon Fixed frame
   * @note Because this is just a DCM, users need to consider the origin of the vector, which you want to convert with this matrix.
   */
  inline const libra::Matrix<3, 3> GetDcmJ2000ToMcmf() const { return dcm_j2000_to_mcmf_; };

 private:
  libra::Matrix<3, 3> dcm_j2000_to_mcmf_;  //!< Direction Cosine Matrix J2000 to MCMF
  libra::Matrix<3, 3> dcm_me_pa_;          //!< DCM from ME (Mean Earth) moon fixed frame to PA (Principal Axis) moon fixed frame

  /**
   * @fn CalcDcmEciToMe
   * @brief Calculate DCM from ECI to ME (Mean Earth) moon fixed frame
   * @param[in] moon_position_eci_m: Moon position vector @ ECI frame [m]
   * @param[in] moon_velocity_eci_m_s: Moon velocity vector @ ECI frame [m/s]
   */
  libra::Matrix<3, 3> CalcDcmEciToMeanEarth(const libra::Vector<3> moon_position_eci_m, const libra::Vector<3> moon_velocity_eci_m_s);
  /**
   * @fn CalcDcmMeToPa
   * @brief Calculate DCM from ME (Mean Earth) moon fixed frame to PA (Principal Axis) moon fixed frame
   */
  libra::Matrix<3, 3> CalcDcmMeanEarthToPrincipalAxis();
};

#endif  // S2E_ENVIRONMENT_GLOBAL_MOON_ROTATION_HPP_
