#include "SimpleCircularOrbit.h"
#include <iostream>
#include <sstream>

SimpleCircularOrbit::SimpleCircularOrbit(double mu, double timestep, int wgs)
  :ODE<N>(timestep), mu(mu)
{
  acc_i_ *= 0;
  if (wgs == 0) { whichconst = wgs72old; }
  else if (wgs == 1) { whichconst = wgs72; }
  else if (wgs == 2) { whichconst = wgs84; }
}

SimpleCircularOrbit::~SimpleCircularOrbit()
{
}

void SimpleCircularOrbit::RHS(double t, const Vector<N>& state, Vector<N>& rhs)
{
  double x = state[0], y = state[1], z = state[2];
  double vx = state[3], vy = state[4], vz = state[5];
  
  double r3 = pow(x*x + y*y + z*z, 1.5);

  rhs[0] = vx; rhs[1] = vy; rhs[2] = vz;
  rhs[3] = acc_i_[0] - mu / r3 * x;
  rhs[4] = acc_i_[1] - mu / r3 * y;
  rhs[5] = acc_i_[2] - mu / r3 * z;
}

void SimpleCircularOrbit::Initialize(Vector<3> init_position, Vector<3> init_velocity, double current_jd, double init_time)
{
  // 状態量ベクトル [x,y,z,vx,vy,vz]
  Vector<N> init_state;
  init_state[0] = init_position[0];
  init_state[1] = init_position[1];
  init_state[2] = init_position[2];
  init_state[3] = init_velocity[0];
  init_state[4] = init_velocity[1];
  init_state[5] = init_velocity[2];
  setup(init_time, init_state);

  // 初期値代入
  acc_i_ *= 0;
  sat_position_i_[0] = init_state[0];
  sat_position_i_[1] = init_state[1];
  sat_position_i_[2] = init_state[2];
  sat_velocity_i_[0] = init_state[3];
  sat_velocity_i_[1] = init_state[4];
  sat_velocity_i_[2] = init_state[5];

  TransECIToGeo(current_jd);
  TransECIToECEF(current_jd);
}

void SimpleCircularOrbit::Propagate(double current_jd)
{
  if (!IsCalcEnabled) return;

  Update(); // ODEの伝播メソッド
  acc_i_ *= 0;
  sat_position_i_[0] = state()[0];
  sat_position_i_[1] = state()[1];
  sat_position_i_[2] = state()[2];
  sat_velocity_i_[0] = state()[3];
  sat_velocity_i_[1] = state()[4];
  sat_velocity_i_[2] = state()[5];

  TransECIToGeo(current_jd);
  TransECIToECEF(current_jd);
}

void SimpleCircularOrbit::AddPositionOffset(Vector<3> offset_i)
{
  auto newstate = state();
  for (auto i = 0; i < 3; i++)
  {
    newstate[i] += offset_i[i];
  }
  setup(x(), newstate);
  sat_position_i_[0] = state()[0];
  sat_position_i_[1] = state()[1];
  sat_position_i_[2] = state()[2];
}

string SimpleCircularOrbit::GetLogHeader() const
{
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

string SimpleCircularOrbit::GetLogValue() const
{
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
