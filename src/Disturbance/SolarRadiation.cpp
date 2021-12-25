//
//  SolarRadiation.cpp
//
//
//  Created by KakiharaKota on 2016/02/26.
//
//

#include "SolarRadiation.h"
#include <math.h>
#include "../Interface/LogOutput/LogUtility.h"

SolarRadiation::SolarRadiation(const Vector<3>& px_arm,
  const Vector<3>& mx_arm,
  const Vector<3>& py_arm,
  const Vector<3>& my_arm,
  const Vector<3>& pz_arm,
  const Vector<3>& mz_arm,
  const Vector<6>& area,
  const Vector<3>& px_normal,
  const Vector<3>& mx_normal,
  const Vector<3>& py_normal,
  const Vector<3>& my_normal,
  const Vector<3>& pz_normal,
  const Vector<3>& mz_normal,
  const Vector<3>& center,
  const Vector<6>& reflectivity,
  const Vector<6>& specularity
) :SurfaceForce(px_arm, mx_arm, py_arm, my_arm, pz_arm, mz_arm, area,
  px_normal, mx_normal, py_normal, my_normal, pz_normal, mz_normal, center)
{
  for (int i = 0; i < 6; ++i)
  {
    reflectivity_[i] = reflectivity[i];
    specularity_[i] = specularity[i];
  }
}

void SolarRadiation::Update(Envir& env, const Spacecraft& spacecraft)
{
  Vector<3> tmp = spacecraft.dynamics_->GetCelestial().GetPosFromSC_b("SUN");
  CalcTorqueForce(tmp, env.srp->CalcTruePressure());
}

// input_b:Direction vector of the sun at the body frame
// item: Solar pressure [N/m^2]
void SolarRadiation::CalcCoef(Vector<3>& input_b, double item)
{
  for (int i = 0; i < 6; i++)
  {//各面で計算
    normal_coef_[i] = area_[i] * item * ((1.0 + reflectivity_[i] * specularity_[i]) * pow(cosX[i], 2.0) + 2.0 / 3.0 * reflectivity_[i] * (1.0 - specularity_[i]) * cosX[i]);
    tangential_coef_[i] = area_[i] * item * (1.0 - reflectivity_[i] * specularity_[i]) * cosX[i] * sinX[i];
  }
}

string SolarRadiation::GetLogHeader() const
{
  string str_tmp = "";

  str_tmp += WriteVector("srptorque", "b", "Nm", 3);
  str_tmp += WriteVector("srpforce", "b", "N", 3);

  return str_tmp;
}

string SolarRadiation::GetLogValue() const
{
  string str_tmp = "";

  str_tmp += WriteVector(torque_b_);
  str_tmp += WriteVector(force_b_);

  return str_tmp;
}
