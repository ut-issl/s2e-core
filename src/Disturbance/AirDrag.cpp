#include "AirDrag.h"

#include <Environment/Global/PhysicalConstants.hpp>
#include <Library/math/Constant.hpp>
#include <cmath>
#include <iostream>

#include "../Interface/InitInput/Initialize.h"
#include "../Interface/LogOutput/LogUtility.h"

using namespace std;
using namespace libra;

AirDrag::AirDrag(const vector<Surface>& surfaces, const Vector<3>& cg_b, const double t_w, const double t_m, const double molecular)
    : SurfaceForce(surfaces, cg_b) {
  int num = surfaces_.size();
  Ct_.assign(num, 1.0);
  Cn_.assign(num, 0.0);
  cnct.assign(num, 0.0);
  Tw_ = t_w;
  Tm_ = t_m;
  M_ = molecular;
};

void AirDrag::Update(const LocalEnvironment& local_env, const Dynamics& dynamics) {
  double air_dens = local_env.GetAtmosphere().GetAirDensity();
  Vector<3> tmp = dynamics.GetOrbit().GetSatVelocity_b();
  CalcTorqueForce(tmp, air_dens);
}

// vel_b:Velocity vector at the body frame[m/s]
// air_dens:air density[kg/m^3]
void AirDrag::CalcCoef(Vector<3>& vel_b, double air_dens) {
  double vel_b_norm_m = norm(vel_b);
  rho_ = air_dens;
  CalCnCt(vel_b);
  for (size_t i = 0; i < surfaces_.size(); i++) {
    double k = 0.5 * rho_ * vel_b_norm_m * vel_b_norm_m * surfaces_[i].GetArea();
    normal_coef_[i] = k * Cn_[i];
    tangential_coef_[i] = k * Ct_[i];
  }
}

double AirDrag::funcPi(double s) {
  double x;
  double erfs = erf(s);  // ERF function is defined in math standard library
  x = s * exp(-s * s) + sqrt(libra::pi) * (s * s + 0.5) * (1.0 + erfs);
  return x;
}

double AirDrag::funcChi(double s) {
  double x;
  double erfs = erf(s);
  x = exp(-s * s) + sqrt(libra::pi) * s * (1.0 + erfs);
  return x;
}

void AirDrag::CalCnCt(Vector<3>& vel_b) {
  double S;
  double vel_b_norm_m = norm(vel_b);
  Vector<3> vel_b_normal(vel_b);
  normalize(vel_b_normal);
  // Re-emitting speed
  S = sqrt(M_ * vel_b_norm_m * vel_b_norm_m / (2.0 * environment::boltzmann_constant_J_K * Tw_));
  // CalcTheta(vel_b);
  for (size_t i = 0; i < surfaces_.size(); i++) {
    double Sn = S * cosX[i];
    double St = S * sinX[i];
    double diffuse = 1.0 - surfaces_[i].GetAirSpecularity();
    Cn_[i] = (2.0 - diffuse) / sqrt(libra::pi) * funcPi(Sn) / (S * S) + diffuse / 2.0 * funcChi(Sn) / (S * S) * sqrt(Tw_ / Tm_);
    Ct_[i] = diffuse * St * funcChi(Sn) / (sqrt(libra::pi) * S * S);
    // for debug
    cnct[i] = Ct_[i] / Cn_[i];
  }
}

void AirDrag::PrintParams(void)  // for debug
{
  Vector<3> arms_b = surfaces_[0].GetPosition();
  cout << "px_arm =(" << arms_b[0] << "," << arms_b[1] << "," << arms_b[2] << ") m \n";
  cout << "area =(" << surfaces_[0].GetArea() << "," << surfaces_[1].GetArea() << "," << surfaces_[2].GetArea() << ") m^2 \n";
  cout << "Temperature =(" << Tw_ << "," << Tm_ << ") K \n";
}

string AirDrag::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("airdrag_torque", "b", "Nm", 3);
  str_tmp += WriteVector("airdrag_force", "b", "N", 3);

  return str_tmp;
}

string AirDrag::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(torque_b_);
  str_tmp += WriteVector(force_b_);

  return str_tmp;
}
