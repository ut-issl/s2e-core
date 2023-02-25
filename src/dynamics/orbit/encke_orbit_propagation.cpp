/**
 * @file encke_orbit_propagation.cpp
 * @brief Class to propagate spacecraft orbit with Encke's method
 */

#include "encke_orbit_propagation.hpp"

#include <library/utilities/macros.hpp>

#include "../../library/orbit/orbital_elements.hpp"

EnckeOrbitPropagation::EnckeOrbitPropagation(const CelestialInformation* celestial_information, const double gravity_constant_m3_s2,
                                             const double propagation_step_s, const double current_time_jd, const libra::Vector<3> position_i_m,
                                             const libra::Vector<3> velocity_i_m_s, const double error_tolerance)
    : Orbit(celestial_information),
      libra::OrdinaryDifferentialEquation<6>(propagation_step_s),
      gravity_constant_m3_s2_(gravity_constant_m3_s2),
      error_tolerance_(error_tolerance),
      propagation_step_s_(propagation_step_s) {
  propagation_time_s_ = 0.0;
  Initialize(current_time_jd, position_i_m, velocity_i_m_s);
}

EnckeOrbitPropagation::~EnckeOrbitPropagation() {}

// Functions for Orbit
void EnckeOrbitPropagation::Propagate(double end_time_s, double current_time_jd) {
  if (!is_calc_enabled_) return;

  // Rectification
  double norm_sat_position_m = norm(spacecraft_position_i_m_);
  double norm_difference_position_m = norm(difference_position_i_m_);
  if (norm_difference_position_m / norm_sat_position_m > error_tolerance_) {
    Initialize(current_time_jd, spacecraft_position_i_m_, spacecraft_velocity_i_m_s_);
  }

  // Update reference orbit
  reference_kepler_orbit.CalcOrbit(current_time_jd);
  reference_position_i_m_ = reference_kepler_orbit.GetPosition_i_m();
  reference_velocity_i_m_s_ = reference_kepler_orbit.GetVelocity_i_m_s();

  // Propagate difference orbit
  SetStepWidth(propagation_step_s_);  // Re-set propagation Δt
  while (end_time_s - propagation_time_s_ - propagation_step_s_ > 1.0e-6) {
    Update();  // Propagation methods of the OrdinaryDifferentialEquation class
    propagation_time_s_ += propagation_step_s_;
  }
  SetStepWidth(end_time_s - propagation_time_s_);  // Adjust the last propagation Δt
  Update();
  propagation_time_s_ = end_time_s;

  difference_position_i_m_[0] = GetState()[0];
  difference_position_i_m_[1] = GetState()[1];
  difference_position_i_m_[2] = GetState()[2];
  difference_velocity_i_m_s_[0] = GetState()[3];
  difference_velocity_i_m_s_[1] = GetState()[4];
  difference_velocity_i_m_s_[2] = GetState()[5];

  UpdateSatOrbit();
}

// Functions for OrdinaryDifferentialEquation
void EnckeOrbitPropagation::DerivativeFunction(double t, const libra::Vector<6>& state, libra::Vector<6>& rhs) {
  UNUSED(t);
  libra::Vector<3> difference_position_i_m_m, difference_acc_i_m_s2;
  for (int i = 0; i < 3; i++) {
    difference_position_i_m_m[i] = state[i];
  }

  double q_func = CalcQFunction(difference_position_i_m_m);
  double r_m = norm(reference_position_i_m_);
  double r_m3 = pow(r_m, 3.0);

  difference_acc_i_m_s2 =
      -(gravity_constant_m3_s2_ / r_m3) * (q_func * spacecraft_position_i_m_ + difference_position_i_m_m) + spacecraft_acceleration_i_m_s2_;

  rhs[0] = state[3];
  rhs[1] = state[4];
  rhs[2] = state[5];
  rhs[3] = difference_acc_i_m_s2[0];
  rhs[4] = difference_acc_i_m_s2[1];
  rhs[5] = difference_acc_i_m_s2[2];
}

// Private Functions
void EnckeOrbitPropagation::Initialize(double current_time_jd, libra::Vector<3> reference_position_i_m, libra::Vector<3> reference_velocity_i_m_s) {
  // General
  fill_up(spacecraft_acceleration_i_m_s2_, 0.0);

  // reference orbit
  reference_position_i_m_ = reference_position_i_m;
  reference_velocity_i_m_s_ = reference_velocity_i_m_s;
  OrbitalElements oe_ref(gravity_constant_m3_s2_, current_time_jd, reference_position_i_m, reference_velocity_i_m_s);
  reference_kepler_orbit = KeplerOrbit(gravity_constant_m3_s2_, oe_ref);

  // difference orbit
  fill_up(difference_position_i_m_, 0.0);
  fill_up(difference_velocity_i_m_s_, 0.0);

  libra::Vector<6> zero(0.0f);
  Setup(0.0, zero);

  UpdateSatOrbit();
}

void EnckeOrbitPropagation::UpdateSatOrbit() {
  spacecraft_position_i_m_ = reference_position_i_m_ + difference_position_i_m_;
  spacecraft_velocity_i_m_s_ = reference_velocity_i_m_s_ + difference_velocity_i_m_s_;

  TransformEciToEcef();
  TransformEcefToGeodetic();
}

double EnckeOrbitPropagation::CalcQFunction(libra::Vector<3> difference_position_i_m) {
  double r2;
  r2 = inner_product(spacecraft_position_i_m_, spacecraft_position_i_m_);

  libra::Vector<3> dr_2r;
  dr_2r = difference_position_i_m - 2.0 * spacecraft_position_i_m_;

  double q = inner_product(difference_position_i_m, dr_2r) / r2;

  double q_func = q * (q * q + 3.0 * q + 3.0) / (pow(1.0 + q, 1.5) + 1.0);

  return q_func;
}
