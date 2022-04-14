#include "Rk4OrbitPropagation.h"

#include <iostream>
#include <sstream>

using std::string;

Rk4OrbitPropagation::Rk4OrbitPropagation(const CelestialInformation* celes_info, double mu, double timestep, Vector<3> init_position,
                                         Vector<3> init_velocity, double current_jd, double init_time)
    : Orbit(celes_info), ODE<N>(timestep), mu(mu) {
  propagate_mode_ = PROPAGATE_MODE::RK4;

  prop_time_ = 0.0;
  prop_step_ = timestep;
  acc_i_ *= 0;

  Initialize(init_position, init_velocity, current_jd, init_time);
}

Rk4OrbitPropagation::~Rk4OrbitPropagation() {}

void Rk4OrbitPropagation::RHS(double t, const Vector<N>& state, Vector<N>& rhs) {
  double x = state[0], y = state[1], z = state[2];
  double vx = state[3], vy = state[4], vz = state[5];

  double r3 = pow(x * x + y * y + z * z, 1.5);

  rhs[0] = vx;
  rhs[1] = vy;
  rhs[2] = vz;
  rhs[3] = acc_i_[0] - mu / r3 * x;
  rhs[4] = acc_i_[1] - mu / r3 * y;
  rhs[5] = acc_i_[2] - mu / r3 * z;

  (void)t;
}

void Rk4OrbitPropagation::Initialize(Vector<3> init_position, Vector<3> init_velocity, double current_jd, double init_time) {
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
  acc_i_ *= 0;
  sat_position_i_[0] = init_state[0];
  sat_position_i_[1] = init_state[1];
  sat_position_i_[2] = init_state[2];
  sat_velocity_i_[0] = init_state[3];
  sat_velocity_i_[1] = init_state[4];
  sat_velocity_i_[2] = init_state[5];

  TransEciToEcef();
  TransEcefToGeo();
}

void Rk4OrbitPropagation::Propagate(double endtime, double current_jd) {
  if (!is_calc_enabled_) return;

  setStepWidth(prop_step_);  // Re-set propagation Δt
  while (endtime - prop_time_ - prop_step_ > 1.0e-6) {
    Update();  // Propagation methods of the ODE class
    prop_time_ += prop_step_;
  }
  setStepWidth(endtime - prop_time_);  // Adjust the last propagation Δt
  Update();
  prop_time_ = endtime;

  acc_i_ *= 0;  // Reset disturbance acceleration
  sat_position_i_[0] = state()[0];
  sat_position_i_[1] = state()[1];
  sat_position_i_[2] = state()[2];
  sat_velocity_i_[0] = state()[3];
  sat_velocity_i_[1] = state()[4];
  sat_velocity_i_[2] = state()[5];

  TransEciToEcef();
  TransEcefToGeo();
}

void Rk4OrbitPropagation::AddPositionOffset(Vector<3> offset_i) {
  auto newstate = state();
  for (auto i = 0; i < 3; i++) {
    newstate[i] += offset_i[i];
  }
  setup(x(), newstate);
  sat_position_i_[0] = state()[0];
  sat_position_i_[1] = state()[1];
  sat_position_i_[2] = state()[2];
}

string Rk4OrbitPropagation::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("sat_position", "i", "m", 3);
  str_tmp += WriteVector("sat_velocity", "i", "m/s", 3);
  str_tmp += WriteVector("sat_velocity", "b", "m/s", 3);
  str_tmp += WriteVector("sat_acc_i", "i", "m/s^2", 3);
  str_tmp += WriteScalar("lat", "rad");
  str_tmp += WriteScalar("lon", "rad");
  str_tmp += WriteScalar("alt", "m");

  return str_tmp;
}

string Rk4OrbitPropagation::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(sat_position_i_, 16);
  str_tmp += WriteVector(sat_velocity_i_, 10);
  str_tmp += WriteVector(sat_velocity_b_);
  str_tmp += WriteVector(acc_i_, 10);
  str_tmp += WriteScalar(lat_rad_);
  str_tmp += WriteScalar(lon_rad_);
  str_tmp += WriteScalar(alt_m_);

  return str_tmp;
}
