/**
 * @file moon_rotation.hpp
 * @brief Class to calculate the moon rotation
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_MOON_ROTATION_HPP_
#define S2E_ENVIRONMENT_GLOBAL_MOON_ROTATION_HPP_

#include "celestial_information.hpp"
#include "math_physics/math/matrix.hpp"
#include "math_physics/math/vector.hpp"
#include "simulation_time.hpp"

namespace s2e::environment {

class CelestialInformation;

/**
 * @enum MoonRotationMode
 * @brief Definition of calculation mode of moon rotation
 */
enum class MoonRotationMode {
  kIdle,     //!< No rotation
  kSimple,   //!< Mean Earth and Principal Axis calculation
  kIauMoon,  //!< IAU_MOON frame given by SPICE
};
/**
 * @fn ConvertMoonRotationMode
 * @brief Convert string to MoonRotationMode
 * @param[in] mode: mode name in string
 */
MoonRotationMode ConvertMoonRotationMode(const std::string mode);

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
  MoonRotation(const CelestialInformation &celestial_information, MoonRotationMode mode = MoonRotationMode::kIdle);

  /**
   * @fn Update
   * @brief Update rotation
   * @param [in] simulation_time: simulation_time
   */
  void Update(const SimulationTime &simulation_time);

  /**
   * @fn GetDcmJ2000ToMcmf
   * @brief Return the DCM between J2000 inertial frame and the Moon Centered Moon Fixed frame
   * @note Because this is just a DCM, users need to consider the origin of the vector, which you want to convert with this matrix.
   */
  inline const math::Matrix<3, 3> GetDcmJ2000ToMcmf() const { return dcm_j2000_to_mcmf_; };

 private:
  MoonRotationMode mode_;                 //!< Rotation mode
  math::Matrix<3, 3> dcm_j2000_to_mcmf_;  //!< Direction Cosine Matrix J2000 to MCMF (Moon Centered Moon Fixed)

  const CelestialInformation &celestial_information_;  //!< Celestial Information to get moon orbit
};

} // namespace s2e::environment

#endif  // S2E_ENVIRONMENT_GLOBAL_MOON_ROTATION_HPP_
