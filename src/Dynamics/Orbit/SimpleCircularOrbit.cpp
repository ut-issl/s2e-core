#include "SimpleCircularOrbit.h"
#include <iostream>
#include <sstream>

SimpleCircularOrbit::SimpleCircularOrbit(const CelestialInformation* celes_info, double mu, double timestep, int wgs)
  : celes_info_(celes_info), ODE<N>(timestep), mu(mu)
{
  prop_time_ = 0.0;
  prop_step_ = timestep;
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

  trans_eci2ecef_ = celes_info_->GetEarthRotation().GetDCMJ2000toXCXF();
  sat_position_ecef_ = trans_eci2ecef_ * sat_position_i_;

  // convert velocity vector in ECI to the vector in ECEF
  Vector<3> OmegaE{ 0.0 }; OmegaE[2] = OmegaEarth;
  Vector<3> wExr = outer_product(OmegaE, sat_position_i_);
  Vector<3> V_wExr = sat_velocity_i_ - wExr;
  sat_velocity_ecef_ = trans_eci2ecef_ * V_wExr;
}

void SimpleCircularOrbit::Propagate(double endtime, double current_jd)
{
  if (!IsCalcEnabled) return;

  setStepWidth(prop_step_); //Re-set propagation Δt
  while (endtime - prop_time_ - prop_step_ > 1.0e-6) 
  {
    Update(); // Propagation methods of the ODE class
    prop_time_ += prop_step_;
  }
  setStepWidth(endtime - prop_time_); //Adjust the last propagation Δt
  Update();
  prop_time_ = endtime;
  
  acc_i_ *= 0; //Reset disturbance acceleration
  sat_position_i_[0] = state()[0];
  sat_position_i_[1] = state()[1];
  sat_position_i_[2] = state()[2];
  sat_velocity_i_[0] = state()[3];
  sat_velocity_i_[1] = state()[4];
  sat_velocity_i_[2] = state()[5];

  TransECIToGeo(current_jd);

  trans_eci2ecef_ = celes_info_->GetEarthRotation().GetDCMJ2000toXCXF();
  sat_position_ecef_ = trans_eci2ecef_ * sat_position_i_;

  // convert velocity vector in ECI to the vector in ECEF
  Vector<3> OmegaE{ 0.0 }; OmegaE[2] = OmegaEarth;
  Vector<3> wExr = outer_product(OmegaE, sat_position_i_);
  Vector<3> V_wExr = sat_velocity_i_ - wExr;
  sat_velocity_ecef_ = trans_eci2ecef_ * V_wExr;
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
