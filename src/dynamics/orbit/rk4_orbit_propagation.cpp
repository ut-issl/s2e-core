/**
 * @file rk4_orbit_propagation.cpp
 * @brief Class to propagate spacecraft orbit with Runge-Kutta-4 method
 */
#include "rk4_orbit_propagation.hpp"

#include <iostream>
#include <sstream>
#include <utilities/macros.hpp>

Rk4OrbitPropagation::Rk4OrbitPropagation(const CelestialInformation* celestial_information, double gravity_constant_m3_s2, double time_step_s,
                                         libra::Vector<3> position_i_m, libra::Vector<3> velocity_i_m_s, double initial_time_s)
    : Orbit(celestial_information), OrdinaryDifferentialEquation<6>(time_step_s), gravity_constant_m3_s2_(gravity_constant_m3_s2) {
  propagate_mode_ = OrbitPropagateMode::kRk4;

  propagation_time_s_ = 0.0;
  propagation_step_s_ = time_step_s;
  spacecraft_acceleration_i_m_s2_ *= 0;

  Initialize(position_i_m, velocity_i_m_s, initial_time_s);
}

Rk4OrbitPropagation::~Rk4OrbitPropagation() {}

void Rk4OrbitPropagation::DerivativeFunction(double t, const libra::Vector<6>& state, libra::Vector<6>& rhs) {
  double x = state[0], y = state[1], z = state[2];
  double vx = state[3], vy = state[4], vz = state[5];

  double r3 = pow(x * x + y * y + z * z, 1.5);

  rhs[0] = vx;
  rhs[1] = vy;
  rhs[2] = vz;
  rhs[3] = spacecraft_acceleration_i_m_s2_[0] - gravity_constant_m3_s2_ / r3 * x;
  rhs[4] = spacecraft_acceleration_i_m_s2_[1] - gravity_constant_m3_s2_ / r3 * y;
  rhs[5] = spacecraft_acceleration_i_m_s2_[2] - gravity_constant_m3_s2_ / r3 * z;

  (void)t;
}

void Rk4OrbitPropagation::Initialize(libra::Vector<3> position_i_m, libra::Vector<3> velocity_i_m_s, double initial_time_s) {
  // state vector [x,y,z,vx,vy,vz]
  libra::Vector<6> init_state;
  init_state[0] = position_i_m[0];
  init_state[1] = position_i_m[1];
  init_state[2] = position_i_m[2];
  init_state[3] = velocity_i_m_s[0];
  init_state[4] = velocity_i_m_s[1];
  init_state[5] = velocity_i_m_s[2];
  Setup(initial_time_s, init_state);

  // initialize
  spacecraft_acceleration_i_m_s2_ *= 0;
  spacecraft_position_i_m_[0] = init_state[0];
  spacecraft_position_i_m_[1] = init_state[1];
  spacecraft_position_i_m_[2] = init_state[2];
  spacecraft_velocity_i_m_s_[0] = init_state[3];
  spacecraft_velocity_i_m_s_[1] = init_state[4];
  spacecraft_velocity_i_m_s_[2] = init_state[5];

  TransformEciToEcef();
  TransformEcefToGeodetic();
}

void Rk4OrbitPropagation::Propagate(const double end_time_s, const double current_time_jd) {
  UNUSED(current_time_jd);

  if (!is_calc_enabled_) return;

  SetStepWidth(propagation_step_s_);  // Re-set propagation Δt
  while (end_time_s - propagation_time_s_ - propagation_step_s_ > 1.0e-6) {
    Update();  // Propagation methods of the OrdinaryDifferentialEquation class
    propagation_time_s_ += propagation_step_s_;
  }
  SetStepWidth(end_time_s - propagation_time_s_);  // Adjust the last propagation Δt
  Update();
  propagation_time_s_ = end_time_s;

  spacecraft_position_i_m_[0] = GetState()[0];
  spacecraft_position_i_m_[1] = GetState()[1];
  spacecraft_position_i_m_[2] = GetState()[2];
  spacecraft_velocity_i_m_s_[0] = GetState()[3];
  spacecraft_velocity_i_m_s_[1] = GetState()[4];
  spacecraft_velocity_i_m_s_[2] = GetState()[5];

  TransformEciToEcef();
  TransformEcefToGeodetic();
}
