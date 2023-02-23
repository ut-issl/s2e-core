/**
 * @file kepler_orbit_propagation.cpp
 * @brief Class to propagate spacecraft orbit with Kepler equation
 */
#include "kepler_orbit_propagation.hpp"

#include <library/utilities/macros.hpp>

#include "../../library/math/s2e_math.hpp"

KeplerOrbitPropagation::KeplerOrbitPropagation(const CelestialInformation* celestial_information, const double current_jd, KeplerOrbit kepler_orbit)
    : Orbit(celestial_information), KeplerOrbit(kepler_orbit) {
  UpdateState(current_jd);
}

KeplerOrbitPropagation::~KeplerOrbitPropagation() {}

void KeplerOrbitPropagation::Propagate(double endtime, double current_jd) {
  UNUSED(endtime);

  if (!is_calc_enabled_) return;

  UpdateState(current_jd);
}

// Private Function
void KeplerOrbitPropagation::UpdateState(const double current_jd) {
  CalcPosVel(current_jd);
  spacecraft_position_i_m_ = position_i_m_;
  spacecraft_velocity_i_m_s_ = velocity_i_m_s_;
  TransEciToEcef();
  TransEcefToGeo();
}
