#include <Library/math/Constant.hpp>

#include "EMDS.h"

EMDS::EMDS(Vector<3> mm, Vector<3> displacement)
  : mm_(mm), dis_(displacement)
{
}

EMDS::~EMDS()
{
}

void EMDS::Update(EMDS& other)
{
  if (!IsCalcEnabled) return;
  Vector<3> results[4];
  calc(pos_, other.pos_, q_, other.q_, dis_, other.dis_, i_ampere, other.i_ampere, results);
  force_b_ = results[0];
  other.force_b_ = results[1];
  torque_b_ = results[2];
  other.torque_b_ = results[3];
}

std::string EMDS::GetLogHeader() const
{
  std::string str_tmp = "";

  str_tmp += WriteVector("emds_force", "b", "N", 3);
  str_tmp += WriteVector("emds_torque", "b", "Nm", 3);

  return str_tmp;
}

std::string EMDS::GetLogValue() const
{
  std::string str_tmp = "";

  str_tmp += WriteVector(force_b_);
  str_tmp += WriteVector(torque_b_);

  return str_tmp;
}

void EMDS::SetParameters(Vector<3> position, Quaternion quaternion, double current)
{
  force_b_ *= 0;
  torque_b_ *= 0;
  pos_ = position;
  q_ = quaternion;
  i_ampere = current;
}

void EMDS::calc(Vector<3> d1_i, Vector<3> d2_i, Quaternion q1_ib, Quaternion q2_ib, 
  Vector<3> r1_b, Vector<3> r2_b, double i1, double i2, Vector<3>* results)
{
  if (i2 >= 0)
  {
    calc_approx(d1_i, d2_i, q1_ib, q2_ib, r1_b, r2_b, i1, i2, 0.23, results);
  }

  else if (i2 < 0)
  {
    calc_approx(d1_i, d2_i, q1_ib, q2_ib, r1_b, r2_b, i1, i2, 0.05, results);
  }

  // 発散しないよう、適当な値でクリップする
  double clipThres = 10.0;
  for (int i = 0; i < 4; i++)
  {
    if (norm(results[i]) >= clipThres)
      results[i] = clipThres * normalize(results[i]);
  }
}

void EMDS::calc_approx(Vector<3> d1_i, Vector<3> d2_i, Quaternion q1_ib, Quaternion q2_ib, 
  Vector<3> r1_b, Vector<3> r2_b, double i1, double i2, double m_c0, Vector<3>* results)
{
  using libra::pi;

  const double mu_0 = 4e-7 * pi;
  Vector<3> m1_mb(0); m1_mb[1] = 4.965; // magnetic moment of neodymium 
  // double m_c0 = 0.23; // offset of coil magnet moment (both 1 and 2)

  double m_cd = (0.3 - m_c0) / 5.0; // Slope of coil magnetic moment change
  Vector<3> m1_cb(0); m1_cb[1] = m_c0 + m_cd*i1;  // Magnetic moment of coil
  Vector<3> m1_b = m1_mb + m1_cb; // Magnetic moment of spacecraft 1
  Vector<3> m2_b(0); m2_b[2] = m_c0 + m_cd*i2;  // Magnetic moment of spacecraft 2
  m1_b *= -1; // 母機側を反転

  // Calculate rotation angle
  double theta1 = 2 * acos(q1_ib[3]);
  double theta2 = 2 * acos(q2_ib[3]);

  // Calculate rotation axis
  Vector<3> n1; n1[0] = q1_ib[0]; n1[1] = q1_ib[1]; n1[2] = q1_ib[2];
  Vector<3> n2; n2[0] = q2_ib[0]; n2[1] = q2_ib[1]; n2[2] = q2_ib[2];
  normalize(n1);
  normalize(n2);

  // Coordinate transformation of magnetic moment
  Vector<3> mu1_i = dot(m1_b, n1)*n1 +
    cos(theta1)*(m1_b - dot(m1_b, n1)*n1) + sin(theta1)*cross(n1, m1_b);
  Vector<3> mu2_i = dot(m2_b, n2)*n2 +
    cos(theta2)*(m2_b - dot(m2_b, n2)*n2) + sin(theta2)*cross(n2, m2_b);

  // Coordinate transformation of position vector
  Vector<3> r1_i = dot(r1_b, n1)*n1 +
    cos(theta1)*(r1_b - dot(r1_b, n1)*n1) + sin(theta1)*cross(n1, r1_b);
  Vector<3> r2_i = dot(r2_b, n2)*n2 +
    cos(theta2)*(r2_b - dot(r2_b, n2)*n2) + sin(theta2)*cross(n2, r2_b);

  // Calculate relative position
  Vector<3> dh = (d2_i + r2_i) - (d1_i + r1_i);
  Vector<3> d = dh; normalize(d);

  // Calculate magnetic force
  Vector<3> F1_i = 3.0*mu_0 / (4.0*pi*pow(norm(-dh), 4))*(dot(mu1_i, -d)*mu2_i + 
    dot(mu2_i, -d)*mu1_i + dot(mu1_i, mu2_i)*(-d) - 5 * dot(mu1_i, -d)*dot(mu2_i, -d)*(-d));
  Vector<3> F2_i = 3.0*mu_0 / (4.0*pi*pow(norm(dh), 4))*(dot(mu2_i, d)*mu1_i +
    dot(mu1_i, d)*mu2_i + dot(mu2_i, mu1_i)*(d)-5 * dot(mu2_i, d)*dot(mu1_i, d)*(d));

  // Calculate the magnet field from magnetic moment
  Vector<3> B1_i = mu_0 / (4 * pi*pow(norm(-dh), 3))*(3 * dot(mu2_i, -d)*(-d) - mu2_i);
  Vector<3> B2_i = mu_0 / (4 * pi*pow(norm(dh), 3))*(3 * dot(mu1_i, d)*(d)-mu1_i);

  // Calculate magnetic torque
  Vector<3> tau1_m_i = cross(mu1_i, B1_i);
  Vector<3> tau2_m_i = cross(mu2_i, B2_i);

  // Calculate force torque
  Vector<3> tau1_F_i = cross(r1_i, F1_i);
  Vector<3> tau2_F_i = cross(r2_i, F2_i);

  //Total the torques
  Vector<3> tau1_i = tau1_m_i + tau1_F_i;
  Vector<3> tau2_i = tau2_m_i + tau2_F_i;

  // Rotate the torque vectors
  Vector<3> tau1_b = dot(tau1_i, n1)*n1 + cos(-theta1)*(tau1_i - dot(tau1_i, n1)*n1) + sin(-theta1)*cross(n1, tau1_i);
  Vector<3> tau2_b = dot(tau2_i, n2)*n2 + cos(-theta2)*(tau2_i - dot(tau2_i, n2)*n2) + sin(-theta2)*cross(n2, tau2_i);

  results[0] = q1_ib.frame_conv(F1_i);
  results[1] = q2_ib.frame_conv(F2_i);
  results[2] = tau1_b;
  results[3] = tau2_b;
}
