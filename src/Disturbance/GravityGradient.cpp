#include "GravityGradient.hpp"

#include <Environment/Global/PhysicalConstants.hpp>
#include <cmath>
#include <fstream>
#include <iostream>

#include "../Interface/LogOutput/LogUtility.h"

using namespace std;

GravityGradient::GravityGradient() : GravityGradient(environment::earth_gravitational_constant_m3_s2) {}

GravityGradient::GravityGradient(const double mu_m3_s2) : mu_m3_s2_(mu_m3_s2) { fill_up(torque_b_, 0.0); }

void GravityGradient::Update(const LocalEnvironment& local_env, const Dynamics& dynamics) {
  CalcTorque(local_env.GetCelesInfo().GetPosFromSC_b("EARTH"), dynamics.GetAttitude().GetInertiaTensor());
}

Vector<3> GravityGradient::CalcTorque(double R0, Vector<3> u_b, Matrix<3, 3> I_b) {
  torque_b_ = 3.0 * mu_m3_s2_ / pow(R0, 3.0) * outer_product(normalize(u_b), I_b * normalize(u_b));
  return torque_b_;
}

Vector<3> GravityGradient::CalcTorque(Vector<3> r_b, Matrix<3, 3> I_b) {
  double coeff = 3.0 * mu_m3_s2_ / pow(norm(r_b), 3.0);
  torque_b_ = coeff * outer_product(normalize(r_b), I_b * normalize(r_b));
  return torque_b_;
}

string GravityGradient::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("ggtorque", "b", "Nm", 3);

  return str_tmp;
}

string GravityGradient::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(torque_b_);

  return str_tmp;
}
