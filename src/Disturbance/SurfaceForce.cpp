#include "SurfaceForce.h"
#include "../Library/math/Vector.hpp"
using libra::Quaternion;
using libra::Vector;

using namespace libra;

SurfaceForce::SurfaceForce(const vector<Surface> &surfaces,
                           const Vector<3> &cg_b)
    : surfaces_(surfaces), cg_b_(cg_b) {
  force_b_ = Vector<3>(0);
  torque_b_ = Vector<3>(0);

  // Initialize vectors
  int num = surfaces_.size();
  normal_coef_.assign(num, 0.0);
  tangential_coef_.assign(num, 0.0);
  cosX.assign(num, 0.0);
  sinX.assign(num, 0.0);
}

// input_b: direction of disturbance source @ body frame
// item: parameter which decide the magnitude of the disturbances (Solar flux,
// air density)
Vector<3> SurfaceForce::CalcTorqueForce(Vector<3> &input_b, double item) {
  CalcTheta(input_b);
  CalcCoef(input_b, item);
  Vector<3> Force(0.0);
  Vector<3> Trq(0.0);
  Vector<3> input_b_normal(input_b);
  normalize(input_b_normal);

  for (size_t i = 0; i < surfaces_.size(); i++) {
    if (cosX[i] >
        0) { // if the surface directs to the disturbance source (sun or air)
      // calc direction of in-plane force
      Vector<3> normal = surfaces_[i].GetNormal();
      Vector<3> ncu = outer_product(input_b_normal, normal);
      Vector<3> ncu_normalized = normalize(ncu);
      Vector<3> s = outer_product(ncu_normalized, normal);
      // calc force
      Vector<3> Fs = -1.0 * normal_coef_[i] * normal + tangential_coef_[i] * s;
      Force += Fs;
      // calc torque
      Vector<3> Ts = outer_product(surfaces_[i].GetPosition() - cg_b_, Fs);
      Trq += Ts;
    }
  }
  force_b_ = Force;
  torque_b_ = Trq;
  return torque_b_;
}

// input_b: direction of disturbance source @ body frame
void SurfaceForce::CalcTheta(Vector<3> &input_b) {
  Vector<3> input_b_normal(input_b);
  normalize(input_b_normal);

  for (size_t i = 0; i < surfaces_.size(); i++) {
    cosX[i] = inner_product(surfaces_[i].GetNormal(), input_b_normal);
    sinX[i] = sqrt(1.0 - cosX[i] * cosX[i]);
  }
}
