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

SolarRadiation::SolarRadiation(const vector<Surface>& surfaces, const Vector<3>& cg_b)
:SurfaceForce(surfaces, cg_b)
{}

void SolarRadiation::Update(const LocalEnvironment & local_env, const Dynamics & dynamics)
{
  Vector<3> tmp = local_env.GetCelesInfo().GetPosFromSC_b("SUN");
  CalcTorqueForce(tmp, local_env.GetSrp().CalcTruePressure());
}

// input_b:Direction vector of the sun at the body frame
// item: Solar pressure [N/m^2]
void SolarRadiation::CalcCoef(Vector<3>& input_b, double item)
{
  for (size_t i = 0; i < surfaces_.size(); i++)
  {//各面で計算
    double area = surfaces_[i].GetArea();
    double reflectivity = surfaces_[i].GetReflectivity();
    double specularity = surfaces_[i].GetSpecularity();
    normal_coef_[i] = area * item * ((1.0 + reflectivity * specularity) * pow(cosX[i], 2.0) + 2.0 / 3.0 * reflectivity * (1.0 - specularity) * cosX[i]);
    tangential_coef_[i] = area * item * (1.0 - reflectivity * specularity) * cosX[i] * sinX[i];
  }
}

std::string SolarRadiation::GetLogHeader() const
{
  std::string str_tmp = "";

  str_tmp += WriteVector("srp_torque", "b", "Nm", 3);
  str_tmp += WriteVector("srp_force", "b", "N", 3);

  return str_tmp;
}

std::string SolarRadiation::GetLogValue() const
{
  std::string str_tmp = "";

  str_tmp += WriteVector(torque_b_);
  str_tmp += WriteVector(force_b_);

  return str_tmp;
}
