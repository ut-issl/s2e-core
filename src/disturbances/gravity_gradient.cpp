/**
 * @file gravity_gradient.cpp
 * @brief Class to calculate the gravity gradient torque
 */

#include "gravity_gradient.hpp"

#include <environment/global/physical_constants.hpp>
#include <cmath>
#include <fstream>
#include <iostream>

#include "../Interface/LogOutput/LogUtility.h"

using namespace std;

GravityGradient::GravityGradient() : GravityGradient(environment::earth_gravitational_constant_m3_s2) {}

GravityGradient::GravityGradient(const double mu_m3_s2) : mu_m3_s2_(mu_m3_s2) { fill_up(torque_b_, 0.0); }

void GravityGradient::Update(const LocalEnvironment& local_env, const Dynamics& dynamics) {
  CalcTorque(local_env.GetCelesInfo().GetCenterBodyPosFromSC_b(),
             dynamics.GetAttitude().GetInertiaTensor());  // TODO: use structure information to get inertia tensor
}

Vector<3> GravityGradient::CalcTorque(const Vector<3> r_b_m, const Matrix<3, 3> I_b_kgm2) {
  double r_norm_m = norm(r_b_m);
  Vector<3> u_b = r_b_m;  // TODO: make undestructive normalize function for Vector
  u_b /= r_norm_m;

  double coeff = 3.0 * mu_m3_s2_ / pow(r_norm_m, 3.0);
  torque_b_ = coeff * outer_product(u_b, I_b_kgm2 * u_b);
  return torque_b_;
}

string GravityGradient::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteVector("gravity_gradient_torque", "b", "Nm", 3);

  return str_tmp;
}

string GravityGradient::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteVector(torque_b_);

  return str_tmp;
}
