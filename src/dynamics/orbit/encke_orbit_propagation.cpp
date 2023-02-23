/**
 * @file encke_orbit_propagation.cpp
 * @brief Class to propagate spacecraft orbit with Encke's method
 */

#include "encke_orbit_propagation.hpp"

#include <library/utilities/macros.hpp>

#include "../../library/orbit/orbital_elements.hpp"

EnckeOrbitPropagation::EnckeOrbitPropagation(const CelestialInformation* celestial_information, const double mu_m3_s2, const double prop_step_s,
                                             const double current_time_jd, const Vector<3> init_position_i_m, const Vector<3> init_velocity_i_m_s,
                                             const double error_tolerance)
    : Orbit(celestial_information), libra::ODE<6>(prop_step_s), mu_m3_s2_(mu_m3_s2), error_tolerance_(error_tolerance), prop_step_s_(prop_step_s) {
  prop_time_s_ = 0.0;
  Initialize(current_time_jd, init_position_i_m, init_velocity_i_m_s);
}

EnckeOrbitPropagation::~EnckeOrbitPropagation() {}

// Functions for Orbit
void EnckeOrbitPropagation::Propagate(double end_time_s, double current_time_jd) {
  if (!is_calc_enabled_) return;

  // Rectification
  double norm_sat_position_m = norm(spacecraft_position_i_m_);
  double norm_diff_position_m = norm(diff_position_i_m_);
  if (norm_diff_position_m / norm_sat_position_m > error_tolerance_) {
    Initialize(current_time_jd, spacecraft_position_i_m_, spacecraft_velocity_i_m_s_);
  }

  // Update reference orbit
  ref_kepler_orbit.CalcPosVel(current_time_jd);
  ref_position_i_m_ = ref_kepler_orbit.GetPosition_i_m();
  ref_velocity_i_m_s_ = ref_kepler_orbit.GetVelocity_i_m_s();

  // Propagate difference orbit
  setStepWidth(prop_step_s_);  // Re-set propagation Δt
  while (end_time_s - prop_time_s_ - prop_step_s_ > 1.0e-6) {
    Update();  // Propagation methods of the ODE class
    prop_time_s_ += prop_step_s_;
  }
  setStepWidth(end_time_s - prop_time_s_);  // Adjust the last propagation Δt
  Update();
  prop_time_s_ = end_time_s;

  diff_position_i_m_[0] = state()[0];
  diff_position_i_m_[1] = state()[1];
  diff_position_i_m_[2] = state()[2];
  diff_velocity_i_m_s_[0] = state()[3];
  diff_velocity_i_m_s_[1] = state()[4];
  diff_velocity_i_m_s_[2] = state()[5];

  UpdateSatOrbit();
}

// Functions for ODE
void EnckeOrbitPropagation::RHS(double t, const Vector<6>& state, Vector<6>& rhs) {
  UNUSED(t);
  Vector<3> diff_pos_i_m, diff_acc_i_m_s2;
  for (int i = 0; i < 3; i++) {
    diff_pos_i_m[i] = state[i];
  }

  double q_func = CalcQFunction(diff_pos_i_m);
  double r_m = norm(ref_position_i_m_);
  double r_m3 = pow(r_m, 3.0);

  diff_acc_i_m_s2 = -(mu_m3_s2_ / r_m3) * (q_func * spacecraft_position_i_m_ + diff_pos_i_m) + spacecraft_acceleration_i_m_s2_;

  rhs[0] = state[3];
  rhs[1] = state[4];
  rhs[2] = state[5];
  rhs[3] = diff_acc_i_m_s2[0];
  rhs[4] = diff_acc_i_m_s2[1];
  rhs[5] = diff_acc_i_m_s2[2];
}

// Private Functions
void EnckeOrbitPropagation::Initialize(double current_time_jd, Vector<3> init_ref_position_i_m, Vector<3> init_ref_velocity_i_m_s) {
  // General
  fill_up(spacecraft_acceleration_i_m_s2_, 0.0);

  // reference orbit
  ref_position_i_m_ = init_ref_position_i_m;
  ref_velocity_i_m_s_ = init_ref_velocity_i_m_s;
  OrbitalElements oe_ref(mu_m3_s2_, current_time_jd, init_ref_position_i_m, init_ref_velocity_i_m_s);
  ref_kepler_orbit = KeplerOrbit(mu_m3_s2_, oe_ref);

  // difference orbit
  fill_up(diff_position_i_m_, 0.0);
  fill_up(diff_velocity_i_m_s_, 0.0);

  Vector<6> zero(0.0f);
  setup(0.0, zero);

  UpdateSatOrbit();
}

void EnckeOrbitPropagation::UpdateSatOrbit() {
  spacecraft_position_i_m_ = ref_position_i_m_ + diff_position_i_m_;
  spacecraft_velocity_i_m_s_ = ref_velocity_i_m_s_ + diff_velocity_i_m_s_;

  TransformEciToEcef();
  TransformEcefToGeodetic();
}

double EnckeOrbitPropagation::CalcQFunction(Vector<3> diff_pos_i) {
  double r2;
  r2 = inner_product(spacecraft_position_i_m_, spacecraft_position_i_m_);

  Vector<3> dr_2r;
  dr_2r = diff_pos_i - 2.0 * spacecraft_position_i_m_;

  double q = inner_product(diff_pos_i, dr_2r) / r2;

  double q_func = q * (q * q + 3.0 * q + 3.0) / (pow(1.0 + q, 1.5) + 1.0);

  return q_func;
}
