/**
 * @file kepler_orbit_propagation.cpp
 * @brief Class to propagate spacecraft orbit with Kepler equation
 */
#include "kepler_orbit_propagation.hpp"

#include <Library/utils/Macros.hpp>

#include "../../Library/math/s2e_math.hpp"

KeplerOrbitPropagation::KeplerOrbitPropagation(const CelestialInformation* celes_info, const double current_jd, KeplerOrbit kepler_orbit)
    : Orbit(celes_info), KeplerOrbit(kepler_orbit) {
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
  sat_position_i_ = position_i_m_;
  sat_velocity_i_ = velocity_i_m_s_;
  TransEciToEcef();
  TransEcefToGeo();
}
