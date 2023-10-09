/**
 * @file moon_rotation.hpp
 * @brief Class to calculate the moon rotation
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_MOON_ROTATION_HPP_
#define S2E_ENVIRONMENT_GLOBAL_MOON_ROTATION_HPP_

#include "library/math/matrix.hpp"

/**
 * @class MoonRotation
 * @brief Class to calculate the moon rotation
 */
class MoonRotation {
 public:
  /**
   * @fn MoonRotation
   * @brief Constructor
   * @param [in] rotation_mode: Designation of rotation model
   */
  MoonRotation(const MoonRotationMode rotation_mode = MoonRotationMode::kSimple);

  /**
   * @fn Update
   * @brief Update rotation
   * @param [in] julian_date: Julian date
   */
  void Update(const double julian_date);

  /**
   * @fn GetDcmJ2000ToXcxf
   * @brief Return the DCM between J2000 inertial frame and the frame of fixed to the target object X (X-Centered X-Fixed)
   */
  inline const libra::Matrix<3, 3> GetDcmJ2000ToXcxf() const { return dcm_j2000_to_xcxf_; };

 private:
  libra::Matrix<3, 3> dcm_j2000_to_xcxf_;  //!< Direction Cosine Matrix J2000 to XCXF(X-Centered X-Fixed)

  MoonRotationMode rotation_mode_;  //!< Designation of dynamics model
};

#endif  // S2E_ENVIRONMENT_GLOBAL_MOON_ROTATION_HPP_
