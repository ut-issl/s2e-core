#include "ThirdBodyGravity.h"

ThirdBodyGravity::ThirdBodyGravity(std::set<string> third_body_list) :third_body_list_(third_body_list)
{
  acceleration_i_ *= 0;
}

ThirdBodyGravity::~ThirdBodyGravity()
{

}

void ThirdBodyGravity::Update(const LocalEnvironment & local_env, const Dynamics & dynamics)
{
  acceleration_i_ = libra::Vector<3>(0); //initialize

  libra::Vector<3> sat_pos_i = dynamics.GetOrbit().GetSatPosition_i();	//position of the spacecraft from the center object
  for (auto third_body : third_body_list_)
  {
    libra::Vector<3> third_body_pos_from_sc_i = local_env.GetCelesInfo().GetPosFromSC_i(third_body.c_str());	//position of the third body from the spacecraft
    libra::Vector<3> third_body_pos_i = sat_pos_i + third_body_pos_from_sc_i;	//position of the third body from the center object
    double gravity_constant = local_env.GetCelesInfo().GetGlobalInfo().GetGravityConstant(third_body.c_str());

    thirdbody_acc_i_ = CalcAcceleration(third_body_pos_i, third_body_pos_from_sc_i, gravity_constant);
    acceleration_i_ += thirdbody_acc_i_;
  }
}

libra::Vector<3> ThirdBodyGravity::CalcAcceleration(libra::Vector<3> s, libra::Vector<3> sr, double GM)
{
  libra::Vector<3> acc;
  double sr_norm = libra::norm(sr);
  double sr_norm3 = sr_norm * sr_norm * sr_norm;
  double s_norm = libra::norm(s);
  double s_norm3 = s_norm * s_norm * s_norm;

  acc = GM * (1.0 / sr_norm3 * sr - 1.0 / s_norm3 * s);

  return acc;
}

string ThirdBodyGravity::GetLogHeader() const
{
  string str_tmp = "";
  str_tmp += WriteVector("acc_thirdbody", "i", "m/s2", 3);

  return str_tmp;
}

string ThirdBodyGravity::GetLogValue() const
{
  string str_tmp = "";
  str_tmp += WriteVector(acceleration_i_);

  return str_tmp;
}