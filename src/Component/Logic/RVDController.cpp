#include "RVDController.h"

RVDController::RVDController(double dt)
  :relPosCtrl(dt), relAttCtrl(dt), relVelCtrl(dt)
{
  relPosCtrl.SetGains(0.8, 20.0, 0.00);
  relVelCtrl.SetGains(100, 0.0, 0.00);
  relAttCtrl.SetGains(3.5, 13.0, 0);
  thrust_b_ *= 0;
  torque_ *= 0;
}

RVDController::~RVDController()
{
}

void RVDController::SetTargetRelPosition(Vector<3> relPos_i)
{
  tar_relpos_i = relPos_i;
}

void RVDController::SetTargetRelVelocity(Vector<3> relVel_i)
{
  tar_relvel_i = relVel_i;
}

void RVDController::SetTargetRelAttitude(Quaternion q12)
{
  tar_q_i2b = q12;
}

void RVDController::SetPositionGain(double p, double d, double i)
{
  relPosCtrl.SetGains(p, d, i);
}

Vector<2> RVDController::CalcCurrent(Vector<3> relpos_i, Quaternion q1_ib, Quaternion q2_ib)
{
  Vector<2> currents;
  currents[0] = 5;
  currents[1] = 5;
  return currents;
}

Vector<6> RVDController::CalcThrust(Vector<3> relpos_now_i, Quaternion q_i2b)
{
  auto input_i = relPosCtrl.CalcOutput(relpos_now_i - tar_relpos_i);
  auto input_b = q_i2b.frame_conv(input_i);
  auto res = CalcThrustEach(input_b);
  return res;
}

Vector<6> RVDController::CalcThrust(Vector<3> relpos_now_i, Vector<3> relvel_now_i, Quaternion q_i2b)
{
  auto input_i = relPosCtrl.CalcOutput(relpos_now_i - tar_relpos_i, relvel_now_i);
  auto input_b = q_i2b.frame_conv(input_i);
  auto res = CalcThrustEach(input_b);
  return res;
}

Vector<6> RVDController::CalcThrustVeloc(Vector<3> relvel_now_i, Quaternion q_i2b)
{
  auto input_i = relVelCtrl.CalcOutput(relvel_now_i - tar_relvel_i);
  auto input_b = q_i2b.frame_conv(input_i);
  auto res = CalcThrustEach(input_b);
  return res;
}

Vector<3> RVDController::CalcRW(Quaternion q_ib_now)
{
  Quaternion q_b2tar = (q_ib_now.conjugate()) * tar_q_i2b;
  double angle = acos_tolerant(q_b2tar[3]);
  Vector<3> trq_dir(0); 
  trq_dir[0] = q_b2tar[0]; trq_dir[1] = q_b2tar[1]; trq_dir[2] = q_b2tar[2];
  normalize(trq_dir);
  auto pdiff = angle * trq_dir;

  auto res = relAttCtrl.CalcOutput(pdiff);

  return res;
}

string RVDController::GetLogHeader() const
{
  string str_tmp = "";
  str_tmp += WriteVector("thrust", "b", "N", 3);
  str_tmp += WriteVector("torque", "b", "Nm", 3);
  return str_tmp;
}

string RVDController::GetLogValue() const
{
  string str_tmp = "";
  str_tmp += WriteVector(thrust_b_);
  str_tmp += WriteVector(torque_);
  return str_tmp;
}

double RVDController::acos_tolerant(double x)
{
  x = (x > 1) ? 1 : x;
  x = (x < -1) ? -1 : x;
  return acos(x);
}

Vector<6> RVDController::CalcThrustEach(Vector<3> thrusts)
{
  const double sf_x = 1;
  const double sf_y = 1;
  const double sf_z = 1;
  Vector<6> results(0);
  int idx[3];
  idx[0] = thrusts[0] > 0 ? MX : PX;
  idx[1] = thrusts[1] > 0 ? MY : PY;
  idx[2] = thrusts[2] > 0 ? MZ : PZ;
  
  for (int i = 0; i < 3; i++)
  {
    int sign = thrusts[i] > 0 ? 1 : -1;
    double thrust = std::abs(thrusts[i]);
    thrust *= sf_x;
    if (thrust > 1) thrust = 1;
    results(idx[i]) = thrust;

    thrust_b_[i] = sign * thrust * 50e-3;
  }
  return results;
}
