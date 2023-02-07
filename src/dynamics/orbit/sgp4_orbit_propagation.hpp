/**
 * @file sgp4_orbit_propagation.hpp
 * @brief Class to propagate spacecraft orbit with SGP4 method with TLE
 */

#ifndef S2E_DYNAMICS_ORBIT_SGP4_ORBIT_PROPAGATION_H_
#define S2E_DYNAMICS_ORBIT_SGP4_ORBIT_PROPAGATION_H_

#include <Library/sgp4/sgp4io.h>
#include <Library/sgp4/sgp4unit.h>

#include <environment/global/celestial_information.hpp>

#include "orbit.hpp"

/**
 * @class Sgp4OrbitPropagation
 * @brief Class to propagate spacecraft orbit with SGP4 method with TLE
 */
class Sgp4OrbitPropagation : public Orbit {
 public:
  /**
   * @fn Sgp4OrbitPropagation
   * @brief Constructor
   * @param [in] celes_info: Celestial information
   * @param [in] tle1: The first line of TLE
   * @param [in] tle2: The second line of TLE
   * @param [in] wgs: Wold Geodetic System
   * @param [in] current_jd: Current Julian day [day]
   */
  Sgp4OrbitPropagation(const CelestialInformation* celes_info, char* tle1, char* tle2, int wgs, double current_jd);

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] endtime: End time of simulation [sec]
   * @param [in] current_jd: Current Julian day [day]
   */
  virtual void Propagate(double endtime, double current_jd);

  /**
   * @fn GetESIOmega
   * @brief Calculate and return ?
   * @note Is this function needed?
   */
  Vector<3> GetESIOmega();

 private:
  gravconsttype whichconst_;                //!< Gravity constant value type
  elsetrec satrec_;                         //!< Structure data for SGP4 library
  const CelestialInformation* celes_info_;  //!< Celestial information
};

#endif  // S2E_DYNAMICS_ORBIT_SGP4_ORBIT_PROPAGATION_H_
