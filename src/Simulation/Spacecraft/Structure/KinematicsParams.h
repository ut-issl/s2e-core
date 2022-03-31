#pragma once

#include <Library/math/Matrix.hpp>
#include <Library/math/Vector.hpp>
using libra::Matrix;
using libra::Vector;

class KinematicsParams {
 public:
  KinematicsParams(Vector<3> cg_b, double mass, Matrix<3, 3> inertia_tensor);
  ~KinematicsParams(){};
  inline const Vector<3>& GetCGb() const { return cg_b_; }
  inline const double& GetMass() const { return mass_; }
  inline const Matrix<3, 3>& GetInertiaTensor() const { return inertia_tensor_; }

 private:
  Vector<3> cg_b_;
  double mass_;
  Matrix<3, 3> inertia_tensor_;
};
