/**
 * @file rk4_orbit_propagation.cpp
 * @brief Class to propagate spacecraft orbit with Runge-Kutta-4 method
 */
#include "rk4_orbit_propagation.hpp"

#include <iostream>
#include <library/utilities/macros.hpp>
#include <sstream>

using std::string;

Rk4OrbitPropagation::Rk4OrbitPropagation(const CelestialInformation* celestial_information, double mu, double timestep, Vector<3> init_position,
                                         Vector<3> init_velocity, double init_time)
    : Orbit(celestial_information), ODE<N>(timestep), mu(mu) {
  propagate_mode_ = OrbitPropagateMode::kRk4;

  prop_time_ = 0.0;
  prop_step_ = timestep;
  spacecraft_acceleration_i_m_s2_ *= 0;

  Initialize(init_position, init_velocity, init_time);
}

Rk4OrbitPropagation::~Rk4OrbitPropagation() {}

void Rk4OrbitPropagation::RHS(double t, const Vector<N>& state, Vector<N>& rhs) {
  double x = state[0], y = state[1], z = state[2];
  double vx = state[3], vy = state[4], vz = state[5];

  double r3 = pow(x * x + y * y + z * z, 1.5);

  rhs[0] = vx;
  rhs[1] = vy;
  rhs[2] = vz;
  rhs[3] = spacecraft_acceleration_i_m_s2_[0] - mu / r3 * x;
  rhs[4] = spacecraft_acceleration_i_m_s2_[1] - mu / r3 * y;
  rhs[5] = spacecraft_acceleration_i_m_s2_[2] - mu / r3 * z;

  (void)t;
}

void Rk4OrbitPropagation::Initialize(Vector<3> init_position, Vector<3> init_velocity, double init_time) {
  // state vector [x,y,z,vx,vy,vz]
  Vector<N> init_state;
  init_state[0] = init_position[0];
  init_state[1] = init_position[1];
  init_state[2] = init_position[2];
  init_state[3] = init_velocity[0];
  init_state[4] = init_velocity[1];
  init_state[5] = init_velocity[2];
  setup(init_time, init_state);

  // initialize
  spacecraft_acceleration_i_m_s2_ *= 0;
  spacecraft_position_i_m_[0] = init_state[0];
  spacecraft_position_i_m_[1] = init_state[1];
  spacecraft_position_i_m_[2] = init_state[2];
  spacecraft_velocity_i_m_s_[0] = init_state[3];
  spacecraft_velocity_i_m_s_[1] = init_state[4];
  spacecraft_velocity_i_m_s_[2] = init_state[5];

  TransEciToEcef();
  TransEcefToGeo();
}

void Rk4OrbitPropagation::Propagate(double endtime, double current_jd) {
  UNUSED(current_jd);

  if (!is_calc_enabled_) return;

  setStepWidth(prop_step_);  // Re-set propagation Δt
  while (endtime - prop_time_ - prop_step_ > 1.0e-6) {
    Update();  // Propagation methods of the ODE class
    prop_time_ += prop_step_;
  }
  setStepWidth(endtime - prop_time_);  // Adjust the last propagation Δt
  Update();
  prop_time_ = endtime;

  spacecraft_position_i_m_[0] = state()[0];
  spacecraft_position_i_m_[1] = state()[1];
  spacecraft_position_i_m_[2] = state()[2];
  spacecraft_velocity_i_m_s_[0] = state()[3];
  spacecraft_velocity_i_m_s_[1] = state()[4];
  spacecraft_velocity_i_m_s_[2] = state()[5];

  TransEciToEcef();
  TransEcefToGeo();
}

void Rk4OrbitPropagation::AddPositionOffset(Vector<3> offset_i) {
  auto newstate = state();
  for (auto i = 0; i < 3; i++) {
    newstate[i] += offset_i[i];
  }
  setup(x(), newstate);
  spacecraft_position_i_m_[0] = state()[0];
  spacecraft_position_i_m_[1] = state()[1];
  spacecraft_position_i_m_[2] = state()[2];
}
