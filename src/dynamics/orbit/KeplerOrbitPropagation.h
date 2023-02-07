/**
 * @file KeplerOrbitPropagation.h
 * @brief Class to propagate spacecraft orbit with Kepler equation
 */
#pragma once
#include "../../Library/Orbit/KeplerOrbit.h"
#include "orbit.hpp"

/**
 * @class KeplerOrbitPropagation
 * @brief Class to propagate spacecraft orbit with Kepler equation
 */
class KeplerOrbitPropagation : public Orbit, public KeplerOrbit {
 public:
  // Initialize with orbital elements
  /**
   * @fn KeplerOrbitPropagation
   * @brief Constructor
   * @param [in] celes_info: Celestial information
   * @param [in] current_jd: Current Julian day [day]
   * @param [in] kepler_orbit: Kepler orbit
   */
  KeplerOrbitPropagation(const CelestialInformation* celes_info, const double current_jd, KeplerOrbit kepler_orbit);
  /**
   * @fn ~KeplerOrbitPropagation
   * @brief Destructor
   */
  ~KeplerOrbitPropagation();

  // Override Orbit
  /**
   * @fn Propagate
   * @brief Propagate orbit
   * @param [in] endtime: End time of simulation [sec]
   * @param [in] current_jd: Current Julian day [day]
   */
  virtual void Propagate(double endtime, double current_jd);

 private:
  /**
   * @fn UpdateState
   * @brief Propagate orbit
   * @param [in] current_jd: Current Julian day [day]
   */
  void UpdateState(const double current_jd);
};
