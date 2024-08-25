/**
 * @file kepler_orbit_propagation.hpp
 * @brief Class to propagate spacecraft orbit with Kepler equation
 */

#ifndef S2E_DYNAMICS_ORBIT_KEPLER_ORBIT_PROPAGATION_HPP_
#define S2E_DYNAMICS_ORBIT_KEPLER_ORBIT_PROPAGATION_HPP_

#include "../../math_physics/orbit/kepler_orbit.hpp"
#include "orbit.hpp"

/**
 * @class KeplerOrbitPropagation
 * @brief Class to propagate spacecraft orbit with Kepler equation
 */
class KeplerOrbitPropagation : public Orbit, public orbit::KeplerOrbit {
 public:
  // Initialize with orbital elements
  /**
   * @fn KeplerOrbitPropagation
   * @brief Constructor
   * @param [in] celestial_information: Celestial information
   * @param [in] current_time_jd: Current Julian day [day]
   * @param [in] kepler_orbit: Kepler orbit
   */
  KeplerOrbitPropagation(const CelestialInformation* celestial_information, const double current_time_jd, orbit::KeplerOrbit kepler_orbit);
  /**
   * @fn ~KeplerOrbitPropagation
   * @brief Destructor
   */
  ~KeplerOrbitPropagation();

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] end_time_s: End time of simulation [sec]
   * @param [in] current_time_jd: Current Julian day [day]
   */
  virtual void Propagate(const double end_time_s, const double current_time_jd);

 private:
  /**
   * @fn UpdateState
   * @brief Propagate orbit
   * @param [in] current_time_jd: Current Julian day [day]
   */
  void UpdateState(const double current_time_jd);
};

#endif  // S2E_DYNAMICS_ORBIT_KEPLER_ORBIT_PROPAGATION_HPP_
