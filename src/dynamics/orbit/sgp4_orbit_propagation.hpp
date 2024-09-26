/**
 * @file sgp4_orbit_propagation.hpp
 * @brief Class to propagate spacecraft orbit with SGP4 method with TLE
 */

#ifndef S2E_DYNAMICS_ORBIT_SGP4_ORBIT_PROPAGATION_HPP_
#define S2E_DYNAMICS_ORBIT_SGP4_ORBIT_PROPAGATION_HPP_

#include <math_physics/orbit/sgp4/sgp4io.h>
#include <math_physics/orbit/sgp4/sgp4unit.h>

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
   * @param [in] celestial_information: Celestial information
   * @param [in] tle1: The first line of TLE
   * @param [in] tle2: The second line of TLE
   * @param [in] wgs_setting: Wold Geodetic System
   * @param [in] current_time_jd: Current Julian day [day]
   */
  Sgp4OrbitPropagation(const CelestialInformation* celestial_information, char* tle1, char* tle2, const int wgs_setting,
                       const double current_time_jd);

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] end_time_s: End time of simulation [sec]
   * @param [in] current_time_jd: Current Julian day [day]
   */
  virtual void Propagate(const double end_time_s, const double current_time_jd);

 private:
  gravconsttype gravity_constant_setting_;  //!< Gravity constant value type
  elsetrec sgp4_data_;                      //!< Structure data for SGP4 library
};

#endif  // S2E_DYNAMICS_ORBIT_SGP4_ORBIT_PROPAGATION_HPP_
