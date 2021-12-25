#define _USE_MATH_DEFINES
#include <math.h>

#include "AirDrag.h"
#include "../Interface/LogOutput/LogUtility.h"
#include "../Interface/InitInput/Initialize.h"

#include <iostream>

using namespace std;
using namespace libra;

AirDrag::AirDrag(const Vector<3>& px_arm,
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
  const Vector<6>& specularity,
  const double t_w,
  const double t_m,
  const double molecular)
  :SurfaceForce(px_arm, mx_arm, py_arm, my_arm, pz_arm, mz_arm, area,
    px_normal, mx_normal, py_normal, my_normal, pz_normal, mz_normal, center)
{
  for (int i = 0; i < 6; ++i)
  {
    reflectivity_[i] = 1.0; //Total reflectance is 1.0 for air drag
    specularity_[i] = specularity[i];//specularity
  }
  Tw_ = t_w;
  Tm_ = t_m;
  M = molecular;
};

void AirDrag::Update(Envir& env, const Spacecraft& spacecraft)
{
  double air_dens = env.atmosphere->GetAirDensity();
  Vector<3> tmp = spacecraft.dynamics_->GetOrbit().GetSatVelocity_b();
  CalcTorqueForce(tmp, air_dens);
}

// vel_b:Velocity vector at the body frame[m/s]
// air_dens:air density[kg/m^3]
void AirDrag::CalcCoef(Vector<3>& vel_b, double air_dens)
{
  double vel_b_norm_m = norm(vel_b);
  rho = air_dens;
  CalCnCt(vel_b);
  for (int i = 0; i < 6; i++)
  {
    double k = 0.5 * rho * vel_b_norm_m * vel_b_norm_m * area_[i];
    normal_coef_[i] = k * Cn[i];
    tangential_coef_[i] = k * Ct[i];
  }
}

double AirDrag::funcPi(double s)
{
  double x;
  double erfs = erf(s); //ERF function is defined in math standard library
  x = s * exp(-s*s) + sqrt(M_PI)*(s*s + 0.5)*(1.0 + erfs);
  return x;
}

double AirDrag::funcChi(double s)
{
  double x;
  double erfs = erf(s);
  x = exp(-s*s) + sqrt(M_PI)*s*(1.0 + erfs);
  return x;
}

void AirDrag::CalCnCt(Vector<3>& vel_b)
{
  double S;
  double vel_b_norm_m = norm(vel_b);
  Vector<3> vel_b_normal(vel_b); 
  normalize(vel_b_normal);
  //Re-emitting speed
  S = sqrt(M * vel_b_norm_m * vel_b_norm_m / (2.0 * K * Tw_));
  //CalcTheta(vel_b);
  for (int i = 0; i < 6; i++)
  {
    double Sn = S * cosX[i];
    double St = S * sinX[i];
    double diffuse = 1.0 - specularity_[i];
    Cn[i] = (2.0 - diffuse) / sqrt(M_PI) * funcPi(Sn) / (S*S) + diffuse / 2.0 * funcChi(Sn) / (S*S) * sqrt(Tw_ / Tm_);
    Ct[i] = diffuse * St * funcChi(Sn) / (sqrt(M_PI) * S*S);
    //for debug
    cnct[i] = Ct[i] / Cn[i];
  }
}

void AirDrag::PrintParams(void)//for debug
{
  cout << "px_arm =(" << arms_b[0][0] << "," << arms_b[0][1] << "," << arms_b[0][2] << ") m \n";
  cout << "area =(" << area_[0] << "," << area_[1] << "," << area_[2] << ") m^2 \n";
  cout << "Temperature =(" << Tw_ << "," << Tm_ << ") K \n";
}

string AirDrag::GetLogHeader() const
{
  string str_tmp = "";

  str_tmp += WriteVector("airdragtorque", "b", "Nm", 3);
  str_tmp += WriteVector("airdragforce", "b", "N", 3);

  return str_tmp;
}

string AirDrag::GetLogValue() const
{
  string str_tmp = "";

  str_tmp += WriteVector(torque_b_);
  str_tmp += WriteVector(force_b_);

  return str_tmp;
}
